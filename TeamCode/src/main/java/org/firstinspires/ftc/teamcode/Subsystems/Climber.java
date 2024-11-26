package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class Climber implements Subsystem {
    //Designating the climberMotor variable to be set in the Climber function
    private final MotorEx climberMotor;

    private final PIDController pidController = new PIDController(0.3, 0, 0);

    private static final double ticksToInches = 1700;

    public enum controlState {
        LIFT_HANG(8),
        MANUAL_HANG(0),
        STOP_HANG(-1);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private controlState currentState = controlState.MANUAL_HANG;

    private double manualPower = 0;

    public Climber() {
        //Linking climberMotor in the code to the motor on the robot
        climberMotor = new MotorEx(hm, "climber", Motor.GoBILDA.RPM_312);

        climberMotor.resetEncoder();

        pidController.setTolerance(1);

        //Setting the configuration for the motor
        climberMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Climber Pos", climberMotor.getCurrentPosition());
//        dashboard.sendTelemetryPacket(packet);

        switch (currentState) {
            case MANUAL_HANG:
                climberMotor.set(manualPower);
                return;
            case LIFT_HANG:
                pidController.setSetPoint(controlState.LIFT_HANG.pos);
                break;
            case STOP_HANG:
                climberMotor.set(0);
                return;
        }
        double currentExtension = Math.abs(climberMotor.getCurrentPosition() / ticksToInches);
        double output = pidController.calculate(currentExtension);

        if (pidController.atSetPoint()) {
            climberMotor.set(0);
        } else {
            climberMotor.set(output);
        }
    }

    public void setPower(double power, controlState state) {
        //Setting the motor to the power in MainTeleop
        currentState = state;

        if (currentState == controlState.MANUAL_HANG) {
            manualPower = power;
        } else if (currentState == controlState.STOP_HANG) {
            climberMotor.set(0);
        }
        else if (currentState == controlState.LIFT_HANG) {
            pidController.setSetPoint(controlState.LIFT_HANG.pos);
        }
    }

}
