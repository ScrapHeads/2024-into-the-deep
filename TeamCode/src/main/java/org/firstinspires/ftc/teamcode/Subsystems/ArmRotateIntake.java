package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmRotateIntake implements Subsystem {
    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armRotateIntake;

    public ArmRotateIntake() {
        //Linking armLift in the code to the motor on the robot
        armRotateIntake = new MotorEx(hm, "armRotateIntake", Motor.GoBILDA.RPM_312);

        //Setting the configuration for the motor
        armRotateIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // add telemetry
    }


    public void setPower(double power) {
        //Setting the lift to the power in MainTeleop
        armRotateIntake.set(power);
    }
}
