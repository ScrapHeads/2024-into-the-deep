package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;

public class liftArm extends CommandBase {
    //linking arm variable to the Arm subsystem
    private final Arm arm;
    //Creating the power to set the motor to
    private final double power;

    public liftArm(Arm arm, double power) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.arm = arm;
        this.power = power;

        // tells the command scheduler what subsystems the command uses
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        //Setting arm to the power set in liftArm function
        arm.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
