package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() { }

    private final MotorEx IntakeMain = new MotorEx("IntakeMain");

    private final MotorEx IntakeControl = new MotorEx("IntakeControl");
    private final ServoEx Gate = new ServoEx("Gate");

    private double IntakeMainPower = 0.0;
    private double IntakeControlPower = 0.0;

    private double GatePosition = 0.5;

    Command IntakeMode = new LambdaCommand()
            .setStart(() -> {
                GatePosition = 0.7;
                IntakeMainPower = -1;
                IntakeControlPower = 1;
    });

    Command Shooting = new LambdaCommand()
            .setStart(() -> {
                GatePosition = 0.5;
                IntakeControlPower = -1;
            });

    @Override
    public void periodic() {
        Gate.setPosition(GatePosition);
        IntakeMain.setPower(IntakeMainPower);
        IntakeControl.setPower(IntakeControlPower);
    }
}