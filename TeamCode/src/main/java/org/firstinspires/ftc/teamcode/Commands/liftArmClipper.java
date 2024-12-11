package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper;

public class liftArmClipper extends CommandBase {
    //linking arm variable to the Arm subsystem
    private final ArmLiftClipper armLiftClipper;
    //Creating the power to set the motor to
    private final double power;

    private ArmLiftClipper.controlState state;

    public liftArmClipper(ArmLiftClipper armLiftClipper, double power, ArmLiftClipper.controlState state) {
        //Taking the inputs from MainTeleop and setting them to the variables inside of this class
        this.armLiftClipper = armLiftClipper;

        this.power = power;

        this.state = state;

        // tells the command scheduler what subsystems the command uses
        addRequirements(armLiftClipper);
    }

    @Override
    public void initialize() {
        //Setting arm to the power set in liftArm function
        armLiftClipper.setPower(power, state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
