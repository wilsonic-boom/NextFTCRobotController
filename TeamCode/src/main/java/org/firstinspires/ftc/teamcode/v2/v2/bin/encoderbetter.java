/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.v2.bin;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Auto Drive By Encoder", group="Robot")
@Disabled
public class encoderbetter extends LinearOpMode {

    enum Alliance { RED, BLUE }
    enum Start { TOP, BOTTOM }

    Alliance alliance;
    Start start;

    private DcMotor TopLeft     = null;
    private DcMotor TopRight    = null;
    private DcMotor BottomLeft  = null;
    private DcMotor BottomRight = null;

    private DcMotor xEncoder;
    private DcMotor yEncoder;
    private IMU imu;
    Orientation angles;

    double FieldX        = 0.0;
    double FieldY        = 0.0;
    double FieldRotation = 0.0;

    int lastXticks = 0;
    int lastYticks = 0;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV  = 537.6;
    static final double DRIVE_GEAR_REDUCTION  = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double CM_TO_INCHES = 1.0 / 2.54;

    // Proportional gains - tune these on the actual robot
    static final double kP_xy  = 0.05;
    static final double kP_rot = 0.01;

    // How close is close enough to move to the next step
    static final double POS_THRESHOLD = 0.5;  // inches
    static final double ROT_THRESHOLD = 2.0;  // degrees

    static final double MAX_POWER = 0.6;

    @Override
    public void runOpMode() {

        start    = Start.BOTTOM;
        alliance = Alliance.BLUE;

        TopLeft     = hardwareMap.get(DcMotor.class, "TopLeft");
        TopRight    = hardwareMap.get(DcMotor.class, "TopRight");
        BottomLeft  = hardwareMap.get(DcMotor.class, "BottomLeft");
        BottomRight = hardwareMap.get(DcMotor.class, "BottomRight");

        xEncoder = hardwareMap.get(DcMotor.class, "xEncoder");
        yEncoder = hardwareMap.get(DcMotor.class, "yEncoder");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        TopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastXticks = 0;
        lastYticks = 0;

        if (opModeIsActive()) {
            main_code();
        }

        while (opModeIsActive()) {
            updateOdometry();
            telemetry.addData("Path", "Complete");
            telemetry.addData("Final Pos", "X: %.2f  Y: %.2f  Rot: %.2f", FieldX, FieldY, FieldRotation);
            telemetry.update();
            sleep(100);
        }
    }

    /*
     * calibrateWithAprilTag
     *
     * Calls AprilTags to scan for a goal tag and get the robot's real field position.
     * AprilTags returns coordinates in cm, so they are converted to inches before
     * updating FieldX, FieldY, FieldRotation. If no tag is found the field position is
     * left unchanged and a telemetry warning is shown.
     *
     * Call this from main_code() whenever the robot is in a spot where a goal tag is visible.
     */
//    public void PositionAprilTag() {
//        apriltagnew.FieldPose pose = apriltagnew.getRobotFieldPose(AprilTagDetection tag);
//
//        if (pose != null) {
//            FieldX        = pose.x * CM_TO_INCHES;
//            FieldY        = pose.y * CM_TO_INCHES;
//            FieldRotation = pose.heading;
//            telemetry.addLine(String.format(
//                    "AprilTag calibration applied: X=%.2f in  Y=%.2f in  Rot=%.2f deg",
//                    FieldX, FieldY, FieldRotation));
//        } else {
//            telemetry.addLine("AprilTag calibration FAILED - no tag found, keeping current position");
//        }
//
//        telemetry.update();
//    }

    /*
     * encoderDrive
     *
     * When fieldcentric = true,  xInches/yInches/turnDegrees are absolute field coordinates to drive to.
     * When fieldcentric = false, xInches/yInches/turnDegrees are relative to the current position.
     *
     * Each loop tick reads the current position from updateOdometry(), computes the error to the
     * target, rotates that error into robot frame, and applies proportional power to all four
     * mecanum wheels. Exits when within threshold or timeout expires.
     */
    public void encoderDrive(double xInches, double yInches,
                             double turnDegrees, boolean fieldcentric,
                             double timeoutS) {

        double targetX, targetY, targetRot;

        if (fieldcentric) {
            targetX   = xInches;
            targetY   = yInches;
            targetRot = turnDegrees;
        } else {
            targetX   = FieldX + xInches;
            targetY   = FieldY + yInches;
            targetRot = FieldRotation + turnDegrees;
        }

        targetRot = targetRot % 360;
        if (targetRot < 0) targetRot += 360;

        if (!opModeIsActive()) return;

        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeoutS) {

            updateOdometry();

            double xError   = targetX   - FieldX;
            double yError   = targetY   - FieldY;
            double rotError = targetRot - FieldRotation;

            // Normalise rotation error to (-180, 180]
            rotError = (rotError + 180) % 360 - 180;

            if (Math.abs(xError)   < POS_THRESHOLD &&
                    Math.abs(yError)   < POS_THRESHOLD &&
                    Math.abs(rotError) < ROT_THRESHOLD) {
                break;
            }

            // Rotate the field-frame position error into robot frame so mecanum
            // mixing works correctly regardless of which way the robot is facing
            double hRad    = Math.toRadians(FieldRotation);
            double rxError =  xError * Math.cos(hRad) + yError * Math.sin(hRad);
            double ryError = -xError * Math.sin(hRad) + yError * Math.cos(hRad);

            double xPow = rxError  * kP_xy;
            double yPow = ryError  * kP_xy;
            double rPow = rotError * kP_rot;

            xPow = Math.max(-MAX_POWER, Math.min(MAX_POWER, xPow));
            yPow = Math.max(-MAX_POWER, Math.min(MAX_POWER, yPow));
            rPow = Math.max(-MAX_POWER, Math.min(MAX_POWER, rPow));

            // logic for mecannum
            double TL = yPow + xPow + rPow;
            double TR = yPow - xPow - rPow;
            double BL = yPow - xPow + rPow;
            double BR = yPow + xPow - rPow;

            // Normalise so no wheel exceeds MAX_POWER
            double maxVal = Math.max(1.0, Math.max(
                    Math.max(Math.abs(TL), Math.abs(TR)),
                    Math.max(Math.abs(BL), Math.abs(BR))
            ));
            TL = (TL / maxVal) * MAX_POWER;
            TR = (TR / maxVal) * MAX_POWER;
            BL = (BL / maxVal) * MAX_POWER;
            BR = (BR / maxVal) * MAX_POWER;

            TopLeft.setPower(TL);
            TopRight.setPower(TR);
            BottomLeft.setPower(BL);
            BottomRight.setPower(BR);

            telemetry.addData("Target",  "X: %.2f  Y: %.2f  Rot: %.2f", targetX, targetY, targetRot);
            telemetry.addData("Current", "X: %.2f  Y: %.2f  Rot: %.2f", FieldX, FieldY, FieldRotation);
            telemetry.addData("Error",   "X: %.2f  Y: %.2f  Rot: %.2f", xError, yError, rotError);
            telemetry.update();
        }

        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);

        // Snap tracked position to exact target to stop small drift errors compounding
        FieldX        = targetX;
        FieldY        = targetY;
        FieldRotation = targetRot;

        sleep(250);
    }

    public void updateOdometry() {
        int xTicks = xEncoder.getCurrentPosition();
        int yTicks = yEncoder.getCurrentPosition();

        int deltaXticks = xTicks - lastXticks;
        int deltaYticks = yTicks - lastYticks;

        lastXticks = xTicks;
        lastYticks = yTicks;

        double dxRobot = deltaXticks / COUNTS_PER_INCH;
        double dyRobot = deltaYticks / COUNTS_PER_INCH;

        angles = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );
        FieldRotation = angles.firstAngle;

        double hRad = Math.toRadians(FieldRotation);

        double dxField = dxRobot * Math.cos(hRad) - dyRobot * Math.sin(hRad);
        double dyField = dxRobot * Math.sin(hRad) + dyRobot * Math.cos(hRad);

        FieldX += dxField;
        FieldY += dyField;
    }

    public double[] FindPos() {
        return new double[]{ FieldX, FieldY, FieldRotation };
    }

    //public static yaw() {
      //  AprilTags.getRobotPoseFromAprilTag() yaw = AprilTags.getyaw;
    //}

    public void main_code() {
        int timeout = 10;
        int change  = 1;
        if (alliance == Alliance.BLUE) {
            change = -1;
        }

        double row_start = -1*(-36.0 + 7.5 + 9);

        if (start == Start.BOTTOM) {

            // Shoot:
            // face tag
            //PositionAprilTag();
            // shoot

            // intake row 1
            encoderDrive(change * row_start, -36, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // Shoot:
            // go to shooting
            // face tag
            //PositionAprilTag();
            // shoot

            // intake row 2
            encoderDrive(change * row_start, -12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // open classifier
            encoderDrive(change * row_start, 0, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);
            encoderDrive(change * -(72.0 - row_start), 0, 0, false, timeout);

            // Shoot:
            encoderDrive(0, 12, 0, false, timeout);
            // go to shooting
            // face tag
            //PositionAprilTag();
            // shoot

            // intake row 3
            encoderDrive(change * row_start, 12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // Shoot:
            // go to shooting
            // face tag
            //PositionAprilTag();
            // shoot

        } else {

            encoderDrive(change * -24, -24, 180, false, timeout);
            // face tag
            //PositionAprilTag();
            // shoot


            // intake row 3
            encoderDrive(change * row_start, 12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // Shoot:
            // go to shooting
            // face tag
            //PositionAprilTag();
            // shoot

            // open classifier
            encoderDrive(change * row_start, 0, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);
            encoderDrive(change * -(72.0 - row_start), 0, 0, false, timeout);

            // intake row 2:
            encoderDrive(change * row_start, -12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // Shoot:
            encoderDrive(change * row_start, -36, 270, true, timeout);
            // go to shooting
            // face tag
            //PositionAprilTag();
            //shoot

            // intake row 1:
            encoderDrive(change * row_start, -36, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);


            // Shoot
            // go to shooting
            // face tag
            //PositionAprilTag();
            // shoot
        }
    }
}