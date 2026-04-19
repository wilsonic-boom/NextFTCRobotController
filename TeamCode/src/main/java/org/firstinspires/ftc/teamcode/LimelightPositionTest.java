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
            Pose3D fieldPose = tag.getRobotPoseFieldSpace();

            telemetry.addLine("───────────────────");
            telemetry.addData("Tag ID",      tag.getFiducialId());
            telemetry.addData("Target area", String.format("%.4f", tag.getTargetArea()));

            if (fieldPose == null) {
                telemetry.addLine("Pose: NULL — tag seen but no pose");
                continue;
            }

            double x       = fieldPose.getPosition().x;
            double y       = fieldPose.getPosition().y;
            double heading = fieldPose.getOrientation().getYaw(AngleUnit.DEGREES);

            telemetry.addData("X",       String.format("%.2f\"", x));
            telemetry.addData("Y",       String.format("%.2f\"", y));
            telemetry.addData("Heading", String.format("%.2f°", heading));
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}