package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Climber;

public class liftClimber extends CommandBase {
    //linking climber variable to the Climber subsystem
    private final Climber climber;
    //Creating the power to set the motor to
    private final double power;

    private Climber.controlState state = null;

    public liftClimber(Climber climber, double power, Climber.controlState state) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.climber = climber;

        this.power = power;

        this.state = state;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        //Setting climber to the power in liftClimber function
        climber.setPower(power, state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
