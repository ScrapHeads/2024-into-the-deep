package org.firstinspires.ftc.teamcode.Commands.Automation;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper.controlState.PICK_UP_CLIPPER;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper.controlState.PLACE_CLIPPER;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper.controlState.RESET_CLIPPER;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.RotateClipperClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ClipperClaw;

public class ClipFirstBlockAuto extends SequentialCommandGroup {

    public ClipFirstBlockAuto(ArmLiftClipper clipperArm, ClipperClaw clipperClaw) {
        addCommands(
            new liftArmClipper(clipperArm, 1, PLACE_CLIPPER),
            new RotateClipperClaw(clipperClaw, closedClaw),
            new WaitCommand(1700),
            new liftArmClipper(clipperArm, 1, PICK_UP_CLIPPER),
            new WaitUntilCommand(() -> clipperArm.isAtPosition(18.5)).andThen(
                new RotateClipperClaw(clipperClaw, openClaw)),
            new liftArmClipper(clipperArm, 1, RESET_CLIPPER)

        );
    }
}
