package org.firstinspires.ftc.teamcode.v2.bin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "shooter_teleop", group = "Linear OpMode")
@Disabled
public class shooter extends LinearOpMode {

    private DcMotorEx motorFL, motorFR, motorBL, motorBR;
    private DcMotorEx shooterMotor;
    private Servo     hoodServo;

    private static final double WHEEL_RADIUS_M   = 0.05;

    private static final double TICKS_PER_REV    = 537.7;

    private static final double HEIGHT_DIFF_M    = 0.3;

    private static final double LAUNCH_ANGLE_DEG = 45.0;

    private static final double G                = 9.81;

    @Override
    public void runOpMode() {

        // ── Init drivetrain ───────────────────────────────────────────────────
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);

        // ── Init shooter ──────────────────────────────────────────────────────
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // enables velocity PID
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);  // flip if needed

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double y   = -gamepad1.left_stick_y;
            double x   =  gamepad1.left_stick_x * 1.1;
            double rx  =  gamepad1.right_stick_x;
            double den =  Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorFL.setPower((y + x + rx) / den);
            motorBL.setPower((y - x + rx) / den);
            motorFR.setPower((y - x - rx) / den);
            motorBR.setPower((y + x - rx) / den);

            if (gamepad1.right_bumper) {
                shootBall(3.0);
            } else {
                shooterMotor.setVelocity(0);
            }

            telemetry.addData("Shooter RPM (actual)", "%.1f",
                    shooterMotor.getVelocity(AngleUnit.DEGREES) / 6.0); // deg/s ÷ 6 = RPM
            telemetry.addData("Hood servo pos",       "%.3f", hoodServo.getPosition());
            telemetry.update();
        }
    }

    private void shootBall(double distanceMeters) {

        // ── 1. Compute required exit velocity ─────────────────────────────────
        double theta    = Math.toRadians(LAUNCH_ANGLE_DEG);
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        // Safety: ensure the trajectory can physically reach the target height.
        double verticalReach = distanceMeters * tanTheta;
        if (verticalReach <= HEIGHT_DIFF_M) {
            telemetry.addData("ERROR", "Angle too shallow for height diff at %.2f m", distanceMeters);
            telemetry.update();
            return;
        }

        double vSquared = (G * distanceMeters * distanceMeters)
                / (2.0 * cosTheta * cosTheta * (verticalReach - HEIGHT_DIFF_M));
        double exitVelocity = Math.sqrt(vSquared);   // m/s

        double omega        = exitVelocity / WHEEL_RADIUS_M;
        double rpm          = omega * 60.0 / (2.0 * Math.PI);
        double ticksPerSec  = rpm * TICKS_PER_REV / 60.0;

        shooterMotor.setVelocity(ticksPerSec);   // DcMotorEx PID handles it

        double servoPosition = (90.0 - LAUNCH_ANGLE_DEG) / 90.0;
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
        hoodServo.setPosition(servoPosition);

        // ── 5. Debug telemetry ────────────────────────────────────────────────
        telemetry.addData("── shootBall ──", "distance = %.2f m", distanceMeters);
        telemetry.addData("Exit velocity",   "%.3f m/s",          exitVelocity);
        telemetry.addData("Target RPM",      "%.1f",              rpm);
        telemetry.addData("Ticks/s sent",    "%.1f",              ticksPerSec);
        telemetry.addData("Launch angle",    "%.1f °",            LAUNCH_ANGLE_DEG);
        telemetry.addData("Hood servo pos",  "%.3f",              servoPosition);
    }
}