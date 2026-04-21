package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Limelight Position Test", group = "Test")
public class LimelightPositionTest extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
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
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("Result: NULL — Limelight not responding");
            telemetry.update();
            return;//
        }

        telemetry.addData("Result valid", result.isValid());
        telemetry.addData("Pipeline",     result.getPipelineIndex());

        if (!result.isValid()) {
            telemetry.addLine("Result: invalid");
            telemetry.update();
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null) {
            telemetry.addLine("Fiducials: NULL list");
            telemetry.update();
            return;
        }

        telemetry.addData("Fiducials found", fiducials.size());

        if (fiducials.isEmpty()) {
            telemetry.addLine("No AprilTag seen");
            telemetry.update();
            return;
        }

        for (LLResultTypes.FiducialResult tag : fiducials) {
            Pose3D camPose = tag.getRobotPoseTargetSpace();
            if (camPose != null) {
                double camX = camPose.getPosition().x;
                double camZ = camPose.getPosition().z;
                double distance2D = Math.sqrt(camX * camX + camZ * camZ);
                telemetry.addData("Distance to tag", String.format("%.2f\"", distance2D));
            }

            telemetry.addLine("───────────────────");
            telemetry.addData("Tag ID",      tag.getFiducialId());
            telemetry.addData("Target area", String.format("%.4f", tag.getTargetArea()));

            // ── Field pose (robot position on field) ──────────────────────────
            Pose3D fieldPose = tag.getRobotPoseFieldSpace();
            if (fieldPose == null) {
                telemetry.addLine("Field pose: NULL — tag seen but no pose");
            } else {
                double x       = fieldPose.getPosition().x;
                double y       = fieldPose.getPosition().y;
                double z       = fieldPose.getPosition().z;
                double heading = fieldPose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addLine("-- Field pose --");
                telemetry.addData("X",       String.format("%.2f\"", x));
                telemetry.addData("Y",       String.format("%.2f\"", y));
                telemetry.addData("Z",       String.format("%.2f\"", z));
                telemetry.addData("Heading", String.format("%.2f°", heading));
            }

            // ── Camera-space pose (distance to tag) ───────────────────────────
            // Z = forward depth, X = left/right, Y = up/down
            // 2D bird's eye distance ignores Y (vertical) and uses X + Z only
            Pose3D camPose1 = tag.getRobotPoseTargetSpace();
            if (camPose1 == null) {
                telemetry.addLine("Cam pose: NULL");
            } else {
                double camX = camPose1.getPosition().x;
                double camY = camPose1.getPosition().y;
                double camZ = camPose1.getPosition().z;
                double distance2D = Math.sqrt(camX * camX + camZ * camZ);
                telemetry.addLine("-- Camera space --");
                telemetry.addData("Cam X",        String.format("%.2f\"", camX));
                telemetry.addData("Cam Y",        String.format("%.2f\"", camY));
                telemetry.addData("Cam Z",        String.format("%.2f\"", camZ));
                telemetry.addData("2D distance",  String.format("%.2f\"", distance2D));
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}