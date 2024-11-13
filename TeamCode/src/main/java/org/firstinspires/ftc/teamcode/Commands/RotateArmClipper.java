package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateClipper;

public class RotateArmClipper extends CommandBase {
    //linking arm variable to the Arm subsystem
    private final ArmRotateClipper armRotateClipper;
    //Creating the power to set the motor to
    private final double power;

    public RotateArmClipper(ArmRotateClipper armRotateClipper, double power) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.armRotateClipper = armRotateClipper;
        this.power = power;

        // tells the command scheduler what subsystems the command uses
        addRequirements(armRotateClipper);
    }

    @Override
    public void initialize() {
        //Setting arm to the power set in liftArm function
        armRotateClipper.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
