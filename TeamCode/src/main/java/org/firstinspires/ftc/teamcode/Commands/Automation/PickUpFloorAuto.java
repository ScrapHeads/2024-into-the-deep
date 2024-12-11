package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;

public class PickUpFloorAuto extends SequentialCommandGroup {
    public PickUpFloorAuto(ArmRotateIntake rotation) {
        addCommands(
                new RotateArmIntake(rotation, 1, PRE_PICK_UP_ROTATE),
                new WaitCommand(1700),
                new RotateArmIntake(rotation, 1, PICK_UP_ROTATE)
        );
    }
}