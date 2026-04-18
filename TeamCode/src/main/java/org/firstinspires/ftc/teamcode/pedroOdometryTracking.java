package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "pedroOdometryTracking", group = "TeleOp")
public class pedroOdometryTracking extends OpMode {
    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor;
    private Limelight3A limelight;
    private Follower follower;

    private static final double LimelightCorrection = 0.15; // every frame april tags seen, deadwheel positions updated 15% closer to predicted from limelight (increasing too much makes it more jittery but u can do ifyw
    private static final double MinimumAccuracy = 0.5; // minimum size of april tag where it is considered good enough to change pos (if far away april tags are not seen, then decrease value, if not increase)

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor = hardwareMap.get(DcMotor.class,
                "BLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");

        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap); // if doesnt work make sure Constants file is correct
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    @Override
    public void loop() {
        follower.update();
        Pose visionPose = getVisionPose(); // uses limelight to see if an april tag is seen and extracts position
        if (visionPose != null) {
            Pose current = follower.getPose();
            double x = lerp(current.getX(), visionPose.getX(), LimelightCorrection);
            double y = lerp(current.getY(), visionPose.getY(), LimelightCorrection);
            follower.setPose(new Pose(x, y, current.getHeading()));
        }

        // razi's field centric dt
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        if (Math.abs(rx) < 0.1) {
            rx = 0;
        }

        double botHeading = follower.getHeading();

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

        FLmotor.setPower(FLpower);
        BLmotor.setPower(BLpower);
        FRmotor.setPower(FRpower);
        BRmotor.setPower(BRpower);

        updateTelemetry();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private Pose getVisionPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        LLResultTypes.FiducialResult tag = fiducials.get(0);
        if (tag.getTargetArea() < MinimumAccuracy) return null;

        // ^^ checks if it is valid to get a position from it
        Pose3D fieldPose = tag.getRobotPoseFieldSpace();
        if (fieldPose == null) return null;

        double x = fieldPose.getPosition().x;
        double y = fieldPose.getPosition().y;
        double heading = fieldPose.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(x, y, heading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    private void updateTelemetry() {
        Pose pose = follower.getPose();

        double headingDeg = Math.toDegrees(pose.getHeading());
        while (headingDeg > 180) headingDeg -= 360;
        while (headingDeg <= -180) headingDeg += 360;

        telemetry.addData("X", String.format("%.2f", pose.getX()));
        telemetry.addData("Y", String.format("%.2f", pose.getY()));
        telemetry.addData("Heading", String.format("%.2f°", headingDeg));
        telemetry.update();
    }

    private static double lerp(double a, double b, double t) { // js used to slightly correct the deadwheels estimated positions (changing t makes this more violent)
        return a + (b - a) * t;
    }
}