package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class Climber implements Subsystem {
    //Designating the climberMotor variable to be set in the Climber function
    private final MotorEx climberMotor;

    public Climber() {
        //Linking climberMotor in the code to the motor on the robot
        climberMotor = new MotorEx(hm, "climber", Motor.GoBILDA.RPM_312);

        //Setting the configuration for the motor
        climberMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        //todo add telemetry
    }

    public void setPower(double power) {
        //Setting the motor to the power in MainTeleop
        climberMotor.set(power);
    }

}
