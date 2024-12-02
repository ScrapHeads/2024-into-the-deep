package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PRE_PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.TUCK_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;

public class PrePlaceHBAuto extends SequentialCommandGroup {
    public PrePlaceHBAuto(ArmLiftIntake lift, ArmRotateIntake rotation) {
        addCommands(
                new RotateArmIntake(rotation, 1, TUCK_ROTATE),
                new WaitUntilCommand(() -> rotation.isAtPosition(5)),
                new liftArmIntake(lift, 1, ArmLiftIntake.controlState.PRE_PLACE_AUTO)
        );
    }
}