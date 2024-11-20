package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class intakeClaw extends CommandBase {
    //linking claw variable to the Claw subsystem
    private final Claw claw;
    //Creating the position to set the servo to
    private final double pos;

    public intakeClaw(Claw claw, double pos) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.claw = claw;
        this.pos = pos;

        //todo add comment of what it does
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        //Setting claw to the position in intakeClaw function
        claw.setPower(pos);
    }

    @Override
    public void end(boolean isInterrupted) {
        claw.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return pos >= 0 && claw.getTouchSensor();
    }
}

