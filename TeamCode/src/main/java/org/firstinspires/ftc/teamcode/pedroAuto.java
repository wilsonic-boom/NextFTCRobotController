package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ShooterCalc;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Date;

// this is bottom left starting position auto code, i can create bottom right for the other alliance by mirroring coords this needs testing tho only has the paths
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class pedroAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DcMotorEx shooter;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        // example how to use for it
        ShooterCalc DataCalc = new ShooterCalc();
        double velocity = DataCalc.lookup(90, "Velocity");
        double hoodPos  = DataCalc.lookup(90, "HoodServoPos");
        // if u have controls for this then:
        ShooterCalc.adjustDistance("tooClose"); // click a button if the ball misses cuz its too close
        ShooterCalc.adjustDistance("tooFar"); // click a button if the ball goes too far
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain one;
        public PathChain two;
        public PathChain three;

        public Paths(Follower follower) {
            one = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(54.879, 10.464),
                                    new Pose(64.660, 38.219),
                                    new Pose(12.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.000, 35.000),
                                    new Pose(32.558, 35.953),
                                    new Pose(55.580, 23.233),
                                    new Pose(60.000, 13.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            two = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 13.000),
                                    new Pose(51.489, 31.096),
                                    new Pose(51.929, 63.649),
                                    new Pose(37.605, 60.091),
                                    new Pose(14.000, 59.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.000, 59.000),
                                    new Pose(14.000, 59.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.000, 59.000),
                                    new Pose(45.627, 68.628),
                                    new Pose(58.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            three = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 83.000),
                                    new Pose(43.325, 94.371),
                                    new Pose(47.534, 80.907),
                                    new Pose(14.000, 83.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.000, 83.000),
                                    new Pose(32.000, 108.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(paths.one);
            pathState = 1;
        }
        else if (pathState == 1 && !follower.isBusy()) {
            shooter.setVelocity(825); // here set correct shooter power and hood servo
            timer.reset();
            pathState = 2;
        }
        else if (pathState == 2 && timer.seconds() > 1) {
            shooter.setVelocity(0);
            follower.followPath(paths.two);
            pathState = 3;
        }
        else if (pathState == 3 && !follower.isBusy()) {
            shooter.setVelocity(1262);  // here set correct shooter power and hood servo
            timer.reset();
            pathState = 4;
        }
        else if (pathState == 4 && timer.seconds() > 1) {
            shooter.setVelocity(0);
            follower.followPath(paths.three);
            pathState = 5;
        }
        else if (pathState == 5 && !follower.isBusy()) {
            shooter.setVelocity(1585);  // here set correct shooter power and hood servo
            timer.reset();
            pathState = 6;
        }
        else if (pathState == 6 && timer.seconds() > 1) {
            shooter.setVelocity(0);
            pathState = 7;
        }

        return pathState;
    }
}