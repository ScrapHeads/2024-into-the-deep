package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.OVER_WALL_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PRE_PICK_UP_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;

public class PickUpFloorAutoSecondSpikeHB extends SequentialCommandGroup {
    public PickUpFloorAutoSecondSpikeHB(ArmRotateIntake rotation) {
        addCommands(
                new RotateArmIntake(rotation, 1, OVER_WALL_ROTATE),
                new WaitCommand(600),
                new RotateArmIntake(rotation, 1, PRE_PICK_UP_ROTATE),
                new WaitCommand(100),
                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE)
        );
    }
}