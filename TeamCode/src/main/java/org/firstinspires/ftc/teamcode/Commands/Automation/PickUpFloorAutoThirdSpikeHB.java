package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PRE_PICK_UP_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;

public class PickUpFloorAutoThirdSpikeHB extends SequentialCommandGroup {
    public PickUpFloorAutoThirdSpikeHB(ArmRotateIntake rotation) {
        addCommands(
                new WaitCommand(750),
                new RotateArmIntake(rotation, 1, PRE_PICK_UP_ROTATE),
                new WaitCommand(1000),
                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE)
        );
    }
}