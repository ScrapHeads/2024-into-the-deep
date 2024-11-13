package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmLiftClipper implements Subsystem {
    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armLiftClipper;

    public ArmLiftClipper() {
        //Linking armLiftClipper in the code to the motor on the robot
        armLiftClipper = new MotorEx(hm, "armLiftClipper", Motor.GoBILDA.RPM_312);

        //Setting the configuration for the motor
        armLiftClipper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // add telemetry
    }

    public void setPower(double power) {
        //Setting the lift to the power in MainTeleop
        armLiftClipper.set(power);
    }
}
