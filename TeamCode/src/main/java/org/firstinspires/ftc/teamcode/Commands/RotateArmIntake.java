package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;

public class RotateArmIntake extends CommandBase {
    //linking arm variable to the Arm subsystem
    private final ArmRotateIntake armRotateIntake;
    //Creating the power to set the motor to
    private final double power;

    public RotateArmIntake(ArmRotateIntake armRotateIntake, double power) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.armRotateIntake = armRotateIntake;
        this.power = power;

        // tells the command scheduler what subsystems the command uses
        addRequirements(armRotateIntake);
    }

    @Override
    public void initialize() {
        //Setting arm to the power set in liftArm function
        armRotateIntake.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
