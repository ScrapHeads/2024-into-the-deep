package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmLiftIntake implements Subsystem {
    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armLiftIntake;

    public ArmLiftIntake() {
        //Linking armLift in the code to the motor on the robot
        armLiftIntake = new MotorEx(hm, "armLiftIntake", Motor.GoBILDA.RPM_312);

        //Setting the configuration for the motor
        armLiftIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // add telemetry
    }

    public void setPower(double power) {
        //Setting the lift to the power in MainTeleop
        armLiftIntake.set(power);
    }

}
