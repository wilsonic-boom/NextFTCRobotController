package org.firstinspires.ftc.teamcode.RobotSystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Claw implements Subsystem {
    public static final org.firstinspires.ftc.teamcode.RobotSystems.Claw INSTANCE = new org.firstinspires.ftc.teamcode.Claw();
    private Claw() { }

    private ServoEx servo = new ServoEx("claw_servo");

    public Command open = new SetPosition(servo, 0.1).requires(this);
    public Command close = new SetPosition(servo, 0.2).requires(this);
}
