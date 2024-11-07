package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmLift;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotate;

public class RotateArm extends CommandBase {
    //linking arm variable to the Arm subsystem
    private final ArmRotate armRotate;
    //Creating the power to set the motor to
    private final double power;

    public RotateArm(ArmRotate armRotate, double power) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.armRotate = armRotate;
        this.power = power;

        // tells the command scheduler what subsystems the command uses
        addRequirements(armRotate);
    }

    @Override
    public void initialize() {
        //Setting arm to the power set in liftArm function
        armRotate.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
