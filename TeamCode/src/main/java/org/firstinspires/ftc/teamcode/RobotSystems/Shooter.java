//package org.firstinspires.ftc.teamcode.RobotSystems;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.utility.LambdaCommand;
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.impl.MotorEx;
//import dev.nextftc.hardware.impl.ServoEx;
//
//public class Shooter implements Subsystem {
//
//    public static final Shooter INSTANCE = new Shooter();
//    private Shooter() {}
//
//    // pd constants
//    public static double KP  = 0.0005;
//    public static double KF  = 0.003;
//    public static double VEL_TOLERANCE = 50.0;
//
//    private final MotorEx shooterMotor = new MotorEx("Shooter");
//    private final ServoEx hood         = new ServoEx("Hood");
//
//    private double targetVelocity = 0.0;
//    private double targetHoodPos  = 0.0;
//
//    @Override
//    public void initialize() {
//        targetVelocity = 0.0;
//        targetHoodPos  = 0.0;
//    }
//
//    @Override
//    public void periodic() {
//        // flywheel pd controller to constantly make sure shooter velocity is the target velocity wanted
//        double currentVel = shooterMotor.getVelocity();
//        double error      = targetVelocity - currentVel;
//        double power      = KF * targetVelocity + KP * error;
//
//        shooterMotor.setPower(Math.max(-1.0, Math.min(power, 1.0)));
//
//        hood.setPosition(targetHoodPos);
//    }
//
//    public boolean isAtTarget() {
//        return Math.abs(shooterMotor.getVelocity() - targetVelocity) < VEL_TOLERANCE;
//    }
//
//    public static double calcVelocity(double x, double y) {
//        return  44.297     * Math.pow(Math.abs(y), 1.0 / 3.0)
//                - 16.893     * Math.abs(x)
//                + 0.0018285  * Math.pow(Math.abs(y), 2.0)
//                + 5.0045e-05 * Math.pow(Math.abs(x), 2.0) * Math.pow(y, 2.0)
//                + 1460.0;
//    }
//
//    public static double calcHoodPosition(double x, double y) {
//        double raw =  0.037524  * Math.pow(Math.abs(y), 0.5)
//                - 0.027099  * Math.pow(Math.abs(x), 1.0 / 3.0)
//                - 0.0051773 * Math.sqrt(x * x + y * y)
//                + 0.18998;
//        return Math.max(0.0, Math.min(raw, 1.0));
//    }
//
//    // set shooter position
//    public Command aimAt(double x, double y) {
//        return new LambdaCommand("AimShooter")
//                .setStart(() -> {
//                    targetVelocity = calcVelocity(x, y);
//                    targetHoodPos  = calcHoodPosition(x, y);
//                })
//                .setIsDone(() -> true)
//                .requires(this);
//    }
//
//    // set shooter velocity to 0
//    public Command stop() {
//        return new LambdaCommand("StopShooter")
//                .setStart(() -> targetVelocity = 0.0)
//                .setIsDone(() -> true)
//                .requires(this);
//    }
//
//    // a command which waits until the shooter is at target position
//    public Command waitUntilReady() {
//        return new LambdaCommand("WaitForShooter")
//                .setIsDone(this::isAtTarget)
//                .requires(this);
//    }
//}