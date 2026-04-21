package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter Flywheel + Servo OpMode
 *
 * Controls:
 *   D-Pad Up    → Increase flywheel target velocity (+50 ticks/sec)
 *   D-Pad Down  → Decrease flywheel target velocity (-50 ticks/sec)
 *   D-Pad Right → Increase servo position (+0.01, max 0.2)
 *   D-Pad Left  → Decrease servo position (-0.01, min 0.0)
 *   Right Bumper → Toggle flywheel on/off
 */
@TeleOp(name = "Shooter OpMode", group = "TeleOp")
public class shooter_test extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx flywheelMotor;
    private Servo     shooterServo;

    // ── PIDF Coefficients ─────────────────────────────────────────────────────
    public static double F = 21.0;
    public static double P = 547.0;
    public static double I = 0.0;
    public static double D = 0.0;

    // ── Flywheel config ───────────────────────────────────────────────────────
    public static double VELOCITY_STEP      = 50.0;   // ticks/sec per d-pad press
    public static double MAX_VELOCITY       = 2800.0; // ticks/sec — adjust to your motor
    public static double MIN_VELOCITY       = 0.0;
    private double       targetVelocity     = 0.0;
    private boolean      flywheelEnabled    = false;

    // ── Servo config ──────────────────────────────────────────────────────────
    public static double SERVO_STEP         = 0.01;
    public static double SERVO_MIN          = 0.0;
    public static double SERVO_MAX          = 0.2;
    private double       servoPosition      = 0.0;

    // ── D-pad debounce ────────────────────────────────────────────────────────
    private final ElapsedTime dpadTimer     = new ElapsedTime();
    private static final double DPAD_DELAY  = 0.15; // seconds between steps

    private boolean prevRightBumper         = false;

    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void runOpMode() {

        // ── Hardware init ──────────────────────────────────────────────────────
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        shooterServo  = hardwareMap.get(Servo.class,     "shooterServo");

        flywheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Apply PIDF to the motor's velocity controller
        flywheelMotor.setVelocityPIDFCoefficients(P, I, D, F);

        // Initialise servo to min position
        shooterServo.setPosition(servoPosition);

        telemetry.addLine("Initialised — waiting for start");
        telemetry.update();

        waitForStart();
        dpadTimer.reset();

        // ── Main loop ──────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            // ── Right bumper: toggle flywheel ──────────────────────────────────
            boolean rightBumper = gamepad1.right_bumper;
            if (rightBumper && !prevRightBumper) {
                flywheelEnabled = !flywheelEnabled;
                if (!flywheelEnabled) {
                    flywheelMotor.setVelocity(0);
                    flywheelMotor.setPower(0);
                }
            }
            prevRightBumper = rightBumper;

            // ── D-pad with debounce ────────────────────────────────────────────
            if (dpadTimer.seconds() >= DPAD_DELAY) {

                if (gamepad1.dpad_up) {
                    targetVelocity = Math.min(targetVelocity + VELOCITY_STEP, MAX_VELOCITY);
                    dpadTimer.reset();

                } else if (gamepad1.dpad_down) {
                    targetVelocity = Math.max(targetVelocity - VELOCITY_STEP, MIN_VELOCITY);
                    dpadTimer.reset();

                } else if (gamepad1.dpad_right) {
                    servoPosition = Math.min(servoPosition + SERVO_STEP, SERVO_MAX);
                    shooterServo.setPosition(servoPosition);
                    dpadTimer.reset();

                } else if (gamepad1.dpad_left) {
                    servoPosition = Math.max(servoPosition - SERVO_STEP, SERVO_MIN);
                    shooterServo.setPosition(servoPosition);
                    dpadTimer.reset();
                }
            }

            // ── Drive flywheel ─────────────────────────────────────────────────
            if (flywheelEnabled && targetVelocity > 0) {
                flywheelMotor.setVelocity(targetVelocity);
            } else if (!flywheelEnabled) {
                flywheelMotor.setVelocity(0);
            }

            // ── Telemetry ──────────────────────────────────────────────────────
            double actualVelocity = flywheelMotor.getVelocity();

            telemetry.addLine("=== Shooter ===");
            telemetry.addData("Flywheel",        flywheelEnabled ? "ENABLED" : "OFF");
            telemetry.addData("Target velocity",  "%.0f ticks/s", targetVelocity);
            telemetry.addData("Actual velocity",  "%.0f ticks/s", actualVelocity);
            telemetry.addData("Velocity error",   "%.0f ticks/s", targetVelocity - actualVelocity);
            telemetry.addLine();
            telemetry.addLine("=== Servo ===");
            telemetry.addData("Servo position",   "%.3f  (range 0.0 – 0.2)", servoPosition);
            telemetry.addLine();
            telemetry.addLine("=== PIDF ===");
            telemetry.addData("P", P);
            telemetry.addData("F", F);
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("  RB         → toggle flywheel");
            telemetry.addLine("  D-Up/Down  → velocity ±50 ticks/s");
            telemetry.addLine("  D-Right/Left → servo ±0.01");
            telemetry.update();
        }

        // ── Cleanup ────────────────────────────────────────────────────────────
        flywheelMotor.setVelocity(0);
        flywheelMotor.setPower(0);
    }
}