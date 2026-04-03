package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.delays.Delay;
import org.firstinspires.ftc.teamcode.mySystems.Lift;
import org.firstinspires.ftc.teamcode.mySystems.Claw;

@Autonomous(name = "visionAprilTags")
public class visionAprilTags extends NextFTCOpMode {
    public visionAprilTags() {
        addComponents(
                new SubsystemComponent(Lift.INSTANCE, Claw.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                Lift.INSTANCE.toHigh,
                new ParallelGroup(
                        Lift.INSTANCE.toMiddle,
                        Claw.INSTANCE.close
                ),
                new Delay(0.5),
                new ParallelGroup(
                        Claw.INSTANCE.open,
                        Lift.INSTANCE.toLow
                )
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
}