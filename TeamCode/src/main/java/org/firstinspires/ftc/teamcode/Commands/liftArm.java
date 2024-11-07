package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmLift;

public class liftArm extends CommandBase {
    //linking arm variable to the Arm subsystem
    private final ArmLift armLift;
    //Creating the power to set the motor to
    private final double power;

    public liftArm(ArmLift armLift, double power) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.armLift = armLift;
        this.power = power;

        // tells the command scheduler what subsystems the command uses
        addRequirements(armLift);
    }

    @Override
    public void initialize() {
        //Setting arm to the power set in liftArm function
        armLift.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}