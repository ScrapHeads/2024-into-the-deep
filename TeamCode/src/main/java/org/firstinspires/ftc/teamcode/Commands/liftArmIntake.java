package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;

public class liftArmIntake extends CommandBase {
    //linking arm variable to the Arm subsystem
    private final ArmLiftIntake armLiftIntake;
    //Creating the power to set the motor to
    private final double power;

    public liftArmIntake(ArmLiftIntake armLiftIntake, double power) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.armLiftIntake = armLiftIntake;
        this.power = power;

        // tells the command scheduler what subsystems the command uses
        addRequirements(armLiftIntake);
    }

    @Override
    public void initialize() {
        //Setting arm to the power set in liftArm function
        armLiftIntake.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
