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

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="encodetemptestNEWNOW", group="Robot")
@Disabled
public class encoder_temp_test extends LinearOpMode {


    enum Alliance {RED_TOP,RED_BOTTOM,BLUE_TOP,BLUE_BOTTOM}


    Alliance alliance;

    /* Declare OpMode members. */
    private DcMotor FLmotor   = null;
    private DcMotor FRmotor  = null;
    private DcMotor BLmotor   = null;
    private DcMotor BRmotor  = null;

    private IMU imu;
    Orientation angles;
    double FieldX = 0.0;   // inches
    double FieldY = 0.0;   // inches
    double FieldRotation = 0.0; // degrees
    int lastXticks = 0;
    int lastYticks = 0;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.

    // LEFT Side: 223 RPM Motor (753.2 ticks/rev)
    static final double     COUNTS_PER_MOTOR_REV_LEFT    = 753.2;
    // RIGHT Side: 312 RPM Motor (537.7 ticks/rev)
    static final double     COUNTS_PER_MOTOR_REV_RIGHT   = 537.7;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_LEFT    = (COUNTS_PER_MOTOR_REV_LEFT * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     COUNTS_PER_INCH_RIGHT   = (COUNTS_PER_MOTOR_REV_RIGHT * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {



        Alliance alliance = Alliance.BLUE_BOTTOM;

        setStartPose();

        // Initialize the drive system variables.
        FLmotor  = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor  = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor  = hardwareMap.get(DcMotor.class, "BLmotor");
        BRmotor  = hardwareMap.get(DcMotor.class, "BRmotor");


        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(imuParams);


        FLmotor.setDirection(DcMotor.Direction.REVERSE);
        BLmotor.setDirection(DcMotor.Direction.REVERSE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
       // leftDrive.setDirection(DcMotor.Direction.REVERSE);
       // rightDrive.setDirection(DcMotor.Direction.FORWARD);

        FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                FLmotor.getCurrentPosition(),
                FRmotor.getCurrentPosition(),
                BLmotor.getCurrentPosition(),
                BRmotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(0,  72, 0,1000);
        telemetry.addData("DRIVING","1");
        encoderDrive(26,  0,0,1000);
        telemetry.addData("DRIVING","2");
        encoderDrive( 0,  -72, 0,1000);
        telemetry.addData("DRIVING","3");
        encoderDrive(-26,  0, 0,1000);
        telemetry.addData("DRIVING","4");

        encoderDrive(15,  32, 180,1000);
        telemetry.addData("DRIVING","5");
        encoderDrive(0,  0, 180,1000);
        telemetry.addData("DRIVING","6");

        while (opModeIsActive()) {

            angles = imu.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES
            );


            telemetry.addData("Deadwheel Encoders","hi");
            telemetry.addData("Path", "Complete");
            telemetry.update();  // pause to display final telemetry message.
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double xInches,
                             double yInches, double turnDegrees,
                             double timeoutS) {
        int TLtarget;
        int TRtarget;
        int BRtarget;
        int BLtarget;
        double speed;
        final double TRACK_WIDTH = 13.025;
        final double WHEEL_BASE = 8.50;
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            double turnRadians = Math.toRadians(turnDegrees);
            double robotRadius = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
            double turnInches = robotRadius * turnRadians;


            double TLinches = yInches + xInches + turnInches;
            double TRinches = yInches - xInches - turnInches;
            double BLinches = yInches - xInches + turnInches;
            double BRinches = yInches + xInches - turnInches;

            speed = 0.5;

            TLtarget = FLmotor.getCurrentPosition() + (int)(TLinches * COUNTS_PER_INCH_LEFT);
            BLtarget = BLmotor.getCurrentPosition() + (int)(BLinches * COUNTS_PER_INCH_LEFT);

            // Right side uses the RIGHT constant
            TRtarget = FRmotor.getCurrentPosition() + (int)(TRinches * COUNTS_PER_INCH_RIGHT);
            BRtarget = BRmotor.getCurrentPosition() + (int)(BRinches * COUNTS_PER_INCH_RIGHT);


            FRmotor.setTargetPosition(TRtarget);
            FLmotor.setTargetPosition(TLtarget);
            BLmotor.setTargetPosition(BLtarget);
            BRmotor.setTargetPosition(BRtarget);

            // Turn On RUN_TO_POSITION
            FLmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            double speedLeft = speed;
            double speedRight = speed * (223.0 / 312.0); // Slow down the 312 RPM motors to match the 223 RPM motors

            FLmotor.setPower(Math.abs(speedLeft));
            BLmotor.setPower(Math.abs(speedLeft));
            FRmotor.setPower(Math.abs(speedRight));
            BRmotor.setPower(Math.abs(speedRight));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FLmotor.isBusy() || FRmotor.isBusy() || BLmotor.isBusy() || BRmotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Starting at",
                        "%7d :%7d :%7d :%7d",
                        FLmotor.getCurrentPosition(),
                        FRmotor.getCurrentPosition(),
                        BLmotor.getCurrentPosition(),
                        BRmotor.getCurrentPosition());

                telemetry.addData("Currently at",
                        "%7d :%7d :%7d :%7d",
                        FLmotor.getCurrentPosition(),
                        FRmotor.getCurrentPosition(),
                        BLmotor.getCurrentPosition(),
                        BRmotor.getCurrentPosition());

            }

            // Stop all motion;
            FLmotor.setPower(0);
            FRmotor.setPower(0);
            BLmotor.setPower(0);
            BRmotor.setPower(0);
            telemetry.addData("TiMEOUTSTOP","TRUE");

            // Turn off RUN_TO_POSITION
            FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.update();
            sleep(250);   // optional pause after each move.
        }
    }
    public double[] FindCR() {

        int colour;
        if (alliance == Alliance.BLUE_BOTTOM || alliance == Alliance.BLUE_TOP) {
            colour = 0;
        } else {
            colour = 1;
        }
        double ROWS[][][] = {{{-48,-36,270},{-48,-12,270},{-48,12,270}},{{48,-36,270},{48,-12,270},{48,12,270}}};
        double cords[] = FindPos();
        double distance = 10000.0;
        int row = 0;
        for (int i = 0; i < 3; i++) {
            double x_distance = cords[0] - ROWS[colour][i][0];
            double y_distance = cords[1] - ROWS[colour][i][1];
            double distancetemp = Math.sqrt(x_distance*x_distance + y_distance*y_distance);
            if (distancetemp < distance) {
                distance = distancetemp;
                row = i;
            }
        }
        return ROWS[colour][row];

    }

    public double[] FindPos() {
    //    double xRobot = xEncoder.getCurrentPosition()/COUNTS_PER_INCH;
    //    double yRobot = yEncoder.getCurrentPosition()/COUNTS_PER_INCH;
    //    double angle = Math.toRadians(angles.firstAngle);
    //    double xField = xRobot * Math.cos(angle) - yRobot * Math.sin(angle);
    //    double yField = xRobot * Math.sin(angle) + yRobot * Math.cos(angle);
    //    double cords[] = {xField,yField, angles.firstAngle};
    //    return cords;
        double cords[] = {FieldX,FieldY,FieldRotation};
        return cords;
    }
    public void setStartPose() {
        if (alliance == Alliance.BLUE_BOTTOM) {
            FieldX = -24.0;
            FieldY = -60.0;
            FieldRotation = 45.0;
        } else if (alliance == Alliance.BLUE_TOP){
            FieldX = -60.0;
            FieldY = 60.0;
            FieldRotation = 135.0;
        } else if (alliance == Alliance.RED_BOTTOM){
            FieldX = 60;
            FieldY = -60.0;
            FieldRotation = 315;
        } else if (alliance == Alliance.RED_TOP){
            FieldX = 60.0;
            FieldY = 60.0;
            FieldRotation = 225.0;
        }
    }
}
