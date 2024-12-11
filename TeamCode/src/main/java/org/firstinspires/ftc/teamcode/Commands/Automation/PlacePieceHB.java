package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.*;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class PlacePieceHB extends SequentialCommandGroup {
    public PlacePieceHB(ArmLiftIntake lift, ArmRotateIntake rotation, Claw claw) {
        addCommands(

                new RotateArmIntake(rotation, 1, PLACE_ROTATE),
                new WaitUntilCommand(() -> rotation.isAtPosition(40)),
                new liftArmIntake(lift, 1, PLACE_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(10)),
                new intakeClaw(claw, 1).withTimeout(500),
                new liftArmIntake(lift, 1, RESET_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(10)),
                new RotateArmIntake(rotation, 1, TUCK_ROTATE)
        );
    }
}
