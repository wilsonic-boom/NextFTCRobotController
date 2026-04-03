package org.firstinspires.ftc.teamcode.RobotSystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Shooter implements Subsystem {
    public static double calculateVelocity(double range) {
        return 8.8912e8 * Math.pow(range, -4)
                - 1.7069e6 * Math.pow(range, -2)
                + 0.0012772 * Math.pow(range, 2) * Math.log(range)
                + 1594.1;
    }

    public static double calculateHoodServo(double range) {
        return 1.7043e5 * Math.pow(range, -4)
                - 26.383 * Math.pow(range, -1)
                - 0.0023591 * range
                + 0.69062;
    }

    public static double velocity(double x, double y) {
        return (44.297 * Math.pow(Math.abs(y), 1.0 / 3.0)
                - 16.893 * Math.abs(x)
                + 0.0018285 * Math.pow(Math.abs(y), 2)
                + 5.0046e-05 * Math.pow(Math.abs(x), 2) * Math.pow(y, 2)
                + 1460); // * (TICKS_PER_REV / 60.0); // maybe the  * (TICKS_PER_REV / 60.0) not needed?
    }

    public static double hoodServoCalc(double x, double y) {
        return 0.037524 * Math.pow(Math.abs(y), 0.5)
                - 0.027099 * Math.pow(Math.abs(x), 1.0 / 3.0)
                - 0.0051773 * Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))
                + 0.18998;
    }

    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    private final MotorEx Shooter = new MotorEx("Shooter");
    private final ServoEx Hood = new ServoEx("Hood");

    private double ShooterPower = 0.0;
    private double ShooterVelocity = 0.0;
    private double HoodPosition = 0.0;

    Command Shooting = new LambdaCommand()
            .setStart(() -> {
                HoodPosition = 0.5; //????
                ShooterVelocity = 1; //??????
            });

    @Override
    public void periodic() {
        Hood.setPosition(HoodPosition);
        Shooter.setPower(ShooterPower);
    }
}