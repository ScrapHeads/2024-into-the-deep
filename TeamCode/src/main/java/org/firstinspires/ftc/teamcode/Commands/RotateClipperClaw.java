package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClipperClaw;

public class RotateClipperClaw extends CommandBase {
    //linking claw variable to the Claw subsystem
    private final ClipperClaw claw;
    //Creating the position to set the servo to
    private final double pos;

    public RotateClipperClaw(ClipperClaw claw, double pos) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.claw = claw;
        this.pos = pos;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        //Setting claw to the position in intakeClaw function
        claw.setPower(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}