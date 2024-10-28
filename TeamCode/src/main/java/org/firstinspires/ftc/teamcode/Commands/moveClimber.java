package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Climber;

public class moveClimber extends CommandBase {
    private final Climber climber;
    private final double power;

    public moveClimber(Climber climber, double power) {
        this.climber = climber;
        this.power = power;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
