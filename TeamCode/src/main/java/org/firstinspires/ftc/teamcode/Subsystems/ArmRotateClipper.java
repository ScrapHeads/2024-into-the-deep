package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmRotateClipper implements Subsystem {
    //Designating the armRotateClipper variable to be set in the Arm function
    private final MotorEx armRotateClipper;

    public ArmRotateClipper() {
        //Linking armRotateClipper in the code to the motor on the robot
        armRotateClipper = new MotorEx(hm, "armRotateClipper", Motor.GoBILDA.RPM_312);

        //Setting the configuration for the motor
        armRotateClipper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // add telemetry
    }


    public void setPower(double power) {
        //Setting the lift to the power in MainTeleop
        armRotateClipper.set(power);
    }
}
