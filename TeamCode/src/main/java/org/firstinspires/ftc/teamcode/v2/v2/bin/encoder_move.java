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
public class encoder_move extends LinearOpMode {

    enum Alliance { RED, BLUE }
    enum Start { TOP, BOTTOM }

    Alliance alliance;
    Start start;

    /* Declare OpMode members. */
    private DcMotor FLmotor    = null;
    private DcMotor FRmotor   = null;
    private DcMotor BLmotor  = null;
    private DcMotor BRmotor = null;

    private DcMotor xEncoder;
    private DcMotor yEncoder;
    private IMU imu;
    Orientation angles;

    double FieldX        = 0.0;  // inches
    double FieldY        = 0.0;  // inches
    double FieldRotation = 0.0;  // degrees

    int lastXticks = 0;
    int lastYticks = 0;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV  = 537.6;
    static final double DRIVE_GEAR_REDUCTION  = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED  = 0.6;
    static final double TURN_SPEED   = 0.5;

    @Override
    public void runOpMode() {

        // Set alliance and start position here
        start    = Start.BOTTOM;
        alliance = Alliance.BLUE;


        FLmotor     = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor    = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor  = hardwareMap.get(DcMotor.class, "BLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");

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

        // Reset drive encoders
        FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized — waiting for Start");
        telemetry.update();

        // START
        waitForStart();

        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastXticks = 0;
        lastYticks = 0;

        if (opModeIsActive()) {
            //main_code();
            encoderDrive(24,24,0,false,10);
            encoderDrive(0,0,180,false,10);
            encoderDrive(24,24,0,false,10);
        }

        // display final position until Stop is pressed
        while (opModeIsActive()) {
            updateOdometry();
            telemetry.addData("Path", "Complete");
            telemetry.addData("Final Pos", "X: %.2f  Y: %.2f  Rot: %.2f",
                    FieldX, FieldY, FieldRotation);
            telemetry.update();
            sleep(100);
        }
    }


    // encoderDrive
    //   xInches      — lateral movement (positive = right)
    //   yInches      — forward/back movement (positive = forward)
    //   turnDegrees  — heading to face (field-centric) or delta turn (relative)
    //   fieldcentric — if true, x/y/turn are absolute field coordinates
    //   timeoutS     — max seconds before giving up
    public void encoderDrive(double xInches, double yInches,
                             double turnDegrees, boolean fieldcentric,
                             double timeoutS) {

        if (fieldcentric) {
            xInches     = xInches     - FieldX;
            yInches     = yInches     - FieldY;
            turnDegrees = turnDegrees - FieldRotation;
        }

        // Normalise turn to (-180, 180]
        turnDegrees = (turnDegrees + 180) % 360 - 180;

        // Update tracked field position
        FieldX        += xInches;
        FieldY        += yInches;
        FieldRotation  = FieldRotation % 360;
        if (FieldRotation < 0) FieldRotation += 360;

        if (!opModeIsActive()) return;

        final double TRACK_WIDTH = 13.025;
        final double WHEEL_BASE  = 8.50;
        final double speed       = 0.5;

        double turnRadians = Math.toRadians(turnDegrees);
        double robotRadius = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
        double turnInches  = robotRadius * turnRadians;

        // Mecanum wheel mixing
        double TLinches = yInches + xInches + turnInches;
        double TRinches = yInches - xInches - turnInches;
        double BLinches = yInches - xInches + turnInches;
        double BRinches = yInches + xInches - turnInches;

        int TLtarget = FLmotor.getCurrentPosition()     + (int)(TLinches * COUNTS_PER_INCH);
        int TRtarget = FRmotor.getCurrentPosition()    + (int)(TRinches * COUNTS_PER_INCH);
        int BLtarget = BLmotor.getCurrentPosition()  + (int)(BLinches * COUNTS_PER_INCH);
        int BRtarget = BRmotor.getCurrentPosition() + (int)(BRinches * COUNTS_PER_INCH);

        FLmotor.setTargetPosition(TLtarget);
        FRmotor.setTargetPosition(TRtarget);
        BLmotor.setTargetPosition(BLtarget);
        BRmotor.setTargetPosition(BRtarget);

        FLmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FLmotor.setPower(Math.abs(speed));
        FRmotor.setPower(Math.abs(speed));
        BLmotor.setPower(Math.abs(speed));
        BRmotor.setPower(Math.abs(speed));

        runtime.reset();

        // runs every tick while motors are moving
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (FLmotor.isBusy() || FRmotor.isBusy() ||
                        BLmotor.isBusy() || BRmotor.isBusy())) {

            // Odometry updated continuously
            updateOdometry();

            telemetry.addData("Field Pos", "X: %.2f  Y: %.2f  Rot: %.2f",
                    FieldX, FieldY, FieldRotation);
            telemetry.addData("Target",     "%7d :%7d :%7d :%7d",
                    TLtarget, TRtarget, BLtarget, BRtarget);
            telemetry.addData("Current",    "%7d :%7d :%7d :%7d",
                    FLmotor.getCurrentPosition(),
                    FRmotor.getCurrentPosition(),
                    BLmotor.getCurrentPosition(),
                    BRmotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        BLmotor.setPower(0);
        BRmotor.setPower(0);

        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        // Rotate robot-frame delta into field frame
        double dxField = dxRobot * Math.cos(hRad) - dyRobot * Math.sin(hRad);
        double dyField = dxRobot * Math.sin(hRad) + dyRobot * Math.cos(hRad);

        FieldX += dxField;
        FieldY += dyField;
    }

    public double[] FindPos() {
        return new double[]{ FieldX, FieldY, FieldRotation };
    }

    public void main_code() {
        int timeout = 10;
        int change  = 1;
        if (alliance == Alliance.BLUE) {
            change = -1;
        }

        double row_start = -36.0 + 7.5 + 9; // = -19.5

        if (start == Start.BOTTOM) {

            // ── Shoot from starting position ──
            // ADD SHOOTING HERE

            // ── Intake row 1 ──
            encoderDrive(change * row_start, -36, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            // ADD HERE GO TO POS
            // ADD HERE SHOOTING

            // ── Intake row 2 ──
            encoderDrive(change * row_start, -12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Open classifier ──
            encoderDrive(change * row_start, 0, 270, true, timeout); // CAN CHANGE X IF NOT NEEDED THAT FAR
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);
            encoderDrive(change * -(72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            encoderDrive(0, 12, 0, false, timeout);
            // ADD SHOOTING HERE

            // ── Intake row 3 ──
            encoderDrive(change * row_start, 12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            // ADD HERE GO TO POS
            // ADD HERE SHOOTING

        } else { // Start.TOP

            // ── Face april tag ──
            encoderDrive(change * -24, -24, 180, false, timeout);

            // ── Calibrate starting position and shoot ──
            // ADD FINDING START AND SHOOTING HERE

            // ── Intake row 3 ──
            encoderDrive(change * row_start, 12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            // ADD GO TO SHOOTING POS AND SHOOT HERE

            // ── Open classifier ──
            encoderDrive(change * row_start, 0, 270, true, timeout); // CAN CHANGE X IF NOT NEEDED THAT FAR
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);
            encoderDrive(change * -(72.0 - row_start), 0, 0, false, timeout);

            // ── Intake row 2 ──
            encoderDrive(change * row_start, -12, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            encoderDrive(change * row_start, -36, 270, true, timeout);
            // ADD GO TO SHOOT POS AND SHOOT HERE

            // ── Intake row 1 ──
            encoderDrive(change * row_start, -36, 270, true, timeout);
            encoderDrive(change * (72.0 - row_start), 0, 0, false, timeout);

            // ── Go to shoot pos and shoot ──
            // ADD GO TO SHOOT POS AND SHOOT HERE
        }
    }
}