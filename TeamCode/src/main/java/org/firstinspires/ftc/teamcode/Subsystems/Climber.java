package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class Climber implements Subsystem {
    private final MotorEx climberMotor;

    public Climber() {
        climberMotor = new MotorEx(hm, "climber", Motor.GoBILDA.RPM_312);
        climberMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // add telemetry
    }

    public void setPower(double power) {
        climberMotor.set(power);
    }

}
