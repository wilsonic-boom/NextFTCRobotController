package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TeleopMaintheg extends LinearOpMode {
    boolean Slow = true;
    int vel = 900;
    double hoodPos = 0;

    // SHOOTER
    double startTime;
    double gateclose = 0.4;
    double gateopen = 0.235;
    private DcMotorEx Shooter = null;
    private DcMotor IntakeMain = null;
    private DcMotor IntakeControl = null;
    private Servo Gate = null;
    private Servo Hood = null;

    double F = 21.0;
    double P = 547.0;

    public double Distance(double X,double Y) {
        // X = Math.abs(X); unneeded
        // Y = Math.abs(Y);

        // if red
        return Math.pow(Math.pow(138 - X, 2) + Math.pow(138 - Y, 2), 0.5);
        // if blue
        // return Math.pow(Math.pow(6 - X, 2) + Math.pow(138 - Y, 2), 0.5);
    }

    @Override
    public void runOpMode() {
        // ----------------------INITIALISING STUFF----------------------------
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // Make sure your ID's match your configuration
        DcMotor FLmotor = hardwareMap.dcMotor.get("FLmotor");
        DcMotor BLmotor = hardwareMap.dcMotor.get("BLmotor");
        DcMotor FRmotor = hardwareMap.dcMotor.get("FRmotor");
        DcMotor BRmotor = hardwareMap.dcMotor.get("BRmotor");

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        IntakeMain = hardwareMap.dcMotor.get("IntakeMain");
        IntakeControl = hardwareMap.dcMotor.get("IntakeControl");

        Hood = hardwareMap.get(Servo.class, "Hood");
        Gate = hardwareMap.get(Servo.class, "Gate");

        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMain.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        Follower follower = Constants.createFollower(hardwareMap); // if doesnt work make sure Constants file is correct
        follower.setStartingPose(new Pose(88, 9, Math.toRadians(90))); // CHANGE

        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Hood.setPosition(0);

        Scheduler.reset();

        telemetry.addLine("INIT done");

        // ----------------------------------------------START----------------------------------------------
        waitForStart();

        imu.resetYaw();
        if (isStopRequested()) return;

        // LOOOOP
        while (opModeIsActive()) {
            // movement
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if (Math.abs(rx) < 0.1) {
                rx = 0;
            }

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double FLpower = (rotY + rotX + rx) / denominator;
            double BLpower = (rotY - rotX + rx) / denominator;
            double FRpower = (rotY - rotX - rx) / denominator;
            double BRpower = (rotY + rotX - rx) / denominator;

            if (gamepad1.rightStickButtonWasPressed()) {
                if (Slow) {
                    Slow = false;
                } else {
                    Slow = true;
                }
            }

            if (Slow) {
                FLmotor.setPower(FLpower);
                BLmotor.setPower(BLpower);
                FRmotor.setPower(FRpower);
                BRmotor.setPower(BRpower);
                telemetry.addData("Speed", "Normal");
            } else {
                FLmotor.setPower(FLpower / 2);
                BLmotor.setPower(BLpower / 2);
                FRmotor.setPower(FRpower / 2);
                BRmotor.setPower(BRpower / 2);
                telemetry.addData("Speed", "Slow");
            }

            // shooter calculations
            ShooterCalc DataCalc = new ShooterCalc();
            vel = (int) DataCalc.lookup(Distance(follower.getPose().getX(), follower.getPose().getY()), "Velocity");
            telemetry.addData("targetVel", vel);
            hoodPos = DataCalc.lookup(Distance(follower.getPose().getX(), follower.getPose().getY()), "HoodServoPos");
            telemetry.addData("targetHood", hoodPos);
            telemetry.addData("estDist", Distance(follower.getPose().getX(), follower.getPose().getY()));

            // telemetry
            telemetry.addData("accVel", Shooter.getVelocity());
            telemetry.addData("hoodPos", Hood.getPosition());

            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("GATE", Gate.getPosition());

            // shooting
            if (!Scheduler.isRunning(ShootRoutine)) {
                telemetry.addData("inside non shooting routine ", "loop");
                if (gamepad1.triangle) {
                    IntakeMain.setDirection(DcMotorSimple.Direction.FORWARD);
                } else {
                    IntakeMain.setDirection(DcMotorSimple.Direction.REVERSE);
                }

                IntakeControl.setPower(1);
                IntakeMain.setPower(1);

                // INTAKE
                if (gamepad1.squareWasPressed()) {
                    if (IntakeMain.getPower() == 1) {
                        IntakeMain.setPower(0);
                        IntakeControl.setPower(0);
                    } else {
                        IntakeMain.setPower(1);
                        IntakeControl.setPower(1);
                    }
                }

                Gate.setPosition(gateclose);

                // Shooter
                if (gamepad1.dpad_up) {
                    double change = Hood.getPosition() + 0.01;
                    if (change > 0.20) {
                        change = 0.20;
                    }
                    Hood.setPosition(change);
                } else if (gamepad1.dpad_down) {
                    Hood.setPosition(Hood.getPosition() - 0.01);
                }

                if (gamepad1.rightBumperWasPressed()) {
                    schedule(ShootRoutine);
                    Scheduler.execute();
                }

                // EMERGENCY STOP
                if (gamepad1.share) {
                    requestOpModeStop();
                }
            }

            telemetry.update();
            follower.update();
            Scheduler.execute();
        }
    }

    public Command ShootRoutine = Command.build()
            .setStart(() -> {
                startTime = System.currentTimeMillis();
                Shooter.setVelocity(vel);
                Hood.setPosition(hoodPos);
            })

            .setExecute(() -> {
                Shooter.setVelocity(vel);
                Hood.setPosition(hoodPos);
                if (Shooter.getVelocity() >= vel - 40 && Shooter.getVelocity() <= vel + 40) {
                    Gate.setPosition(gateopen);
                    IntakeControl.setDirection(DcMotorSimple.Direction.REVERSE);
                    IntakeControl.setPower(1);

                }
            })

            .setDone(() -> System.currentTimeMillis() - startTime > 2000)

            .setEnd(endCondition -> {
                Gate.setPosition(gateclose);
                IntakeControl.setDirection(DcMotorSimple.Direction.FORWARD);
                IntakeControl.setPower(1);
            });
}