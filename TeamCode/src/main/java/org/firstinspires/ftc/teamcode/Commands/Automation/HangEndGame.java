package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.HANG_HIGH_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PRE_HANG_HIGH_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.Climber.controlState.HANG_THREE;
import static org.firstinspires.ftc.teamcode.Subsystems.Climber.controlState.HANG_TWO;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Commands.liftClimber;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;

public class HangEndGame extends SequentialCommandGroup {
    public HangEndGame(ArmLiftIntake lift, ArmRotateIntake rotation, Climber climber) {
        addCommands(
                //Level one hang
                new liftClimber(climber, 1, HANG_TWO),
                new WaitUntilCommand(() -> climber.isAtPosition(.5)),

                //Pre to level two
                new RotateArmIntake(rotation, 1, PRE_HANG_HIGH_ROTATE),
                new WaitUntilCommand(() -> rotation.isAtPosition(5)),
                new liftArmIntake(lift, 1, ArmLiftIntake.controlState.PRE_HANG_HIGH_LIFT),
                new WaitUntilCommand(() -> lift.isAtPosition(.7)),

                //Level two
                new RotateArmIntake(rotation, 1, HANG_HIGH_ROTATE),
                new WaitUntilCommand(() -> rotation.isAtPosition(3)),
                new liftArmIntake(lift,1 , ArmLiftIntake.controlState.HANG_HIGH_LIFT),
                new liftClimber(climber, 1, HANG_THREE)
        );
    }
}