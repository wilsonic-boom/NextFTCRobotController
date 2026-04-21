package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Limelight MT2 Pose Test", group = "Test")
public class LimelightMT2Test extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.addLine("Limelight initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();

        // Set your AprilTag pipeline
        limelight.pipelineSwitch(0);
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("Result: NULL (no data from Limelight)");
            telemetry.update();
            return;
        }

        telemetry.addData("Pipeline", result.getPipelineIndex());
        telemetry.addData("Valid", result.isValid());

        if (!result.isValid()) {
            telemetry.addLine("No valid targets");
            telemetry.update();
            return;
        }

        Pose3D botPose = result.getBotpose();

        if (botPose == null) {
            telemetry.addLine("MT2 Pose: NULL (no solve)");
        } else {
            double x = botPose.getPosition().x;
            double y = botPose.getPosition().y;
            double z = botPose.getPosition().z;

            double heading = botPose.getOrientation().getYaw(AngleUnit.DEGREES);

            telemetry.addLine("=== MegaTag2 Pose ===");
            telemetry.addData("X (in)", "%.2f", x);
            telemetry.addData("Y (in)", "%.2f", y);
            telemetry.addData("Z (in)", "%.2f", z);
            telemetry.addData("Heading (deg)", "%.2f", heading);
        }

        // =========================
        // 👁 OPTIONAL: TAG INFO
        // =========================
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags != null && !tags.isEmpty()) {
            telemetry.addLine("=== Visible Tags ===");
            telemetry.addData("Count", tags.size());

            for (LLResultTypes.FiducialResult tag : tags) {
                telemetry.addLine("-------------------");
                telemetry.addData("Tag ID", tag.getFiducialId());
                telemetry.addData("Area", "%.4f", tag.getTargetArea());
            }
        } else {
            telemetry.addLine("No AprilTags detected");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}