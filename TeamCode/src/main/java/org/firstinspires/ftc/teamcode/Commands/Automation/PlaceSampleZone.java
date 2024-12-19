package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PRE_PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.TUCK_ROTATE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class PlaceSampleZone extends SequentialCommandGroup {
    public PlaceSampleZone(ArmRotateIntake rotation, Claw claw) {
        addCommands(
                new RotateArmIntake(rotation, 1, PRE_PICK_UP_ROTATE),
                new WaitUntilCommand(() -> rotation.isAtPosition(15)),
                new intakeClaw(claw, 1).withTimeout(200),
                new RotateArmIntake(rotation, 1, TUCK_ROTATE)
        );
    }
}