package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmLiftClipper implements Subsystem {
    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armLiftClipper;

    //TODO get ticks to inches
    private static final double ticksToInches = -115.5;

    private final PIDController pidController = new PIDController(0.3, 0, 0);

    public enum controlState {
        MANUAL_CLIPPER(-1),
        HOLD_CLIPPER(-2),
        PICK_UP_CLIPPER(3.5),
        PLACE_CLIPPER(21.5);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private ArmLiftClipper.controlState currentState = controlState.MANUAL_CLIPPER;

    private double manualPower = 0;
    private double savedPosition = 0;

    public ArmLiftClipper() {
        //Linking armLiftClipper in the code to the motor on the robot
        armLiftClipper = new MotorEx(hm, "armLiftClipper", Motor.GoBILDA.RPM_312);

        armLiftClipper.resetEncoder();

        //Setting the configuration for the motor
        armLiftClipper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        pidController.setTolerance(.5);
    }

    @Override
    public void periodic() {
        // add telemetry
        switch (currentState) {
            case MANUAL_CLIPPER:
                setPower(manualPower, controlState.MANUAL_CLIPPER);
                return;
            case HOLD_CLIPPER:
                pidController.setSetPoint(savedPosition);
                break;
            case PLACE_CLIPPER:
                pidController.setSetPoint(controlState.PLACE_CLIPPER.pos);
                break;
            case PICK_UP_CLIPPER:
                pidController.setSetPoint(controlState.PICK_UP_CLIPPER.pos);
                break;
        }

        double currentExtension = Math.abs(armLiftClipper.getCurrentPosition() / ticksToInches);
        double output = -pidController.calculate(currentExtension);

        if (pidController.atSetPoint()) {
            armLiftClipper.set(0);
        } else {
            armLiftClipper.set(output);
        }
    }

    public void setPower(double power, controlState state) {
        //Setting the lift to the power in MainTeleop

        double currentExtension = Math.abs(armLiftClipper.getCurrentPosition() / ticksToInches);

        currentState = state;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Current Extension", currentExtension);
        packet.put("Current Extension", armLiftClipper.getCurrentPosition());
        packet.put("What state", state);
        dashboard.sendTelemetryPacket(packet);

        if (currentState == controlState.MANUAL_CLIPPER) {
            armLiftClipper.set(power);
            manualPower = power;
        } else if (currentState == controlState.PLACE_CLIPPER) {
            pidController.setSetPoint(controlState.PLACE_CLIPPER.pos);
        }
        else if (currentState == controlState.PICK_UP_CLIPPER) {
            pidController.setSetPoint(controlState.PICK_UP_CLIPPER.pos);
        } else {
            armLiftClipper.set(0);
            savedPosition = currentExtension;
        }
    }

    public boolean isAtPosition(double tolerance) {
        double curPos = armLiftClipper.getCurrentPosition() / ticksToInches;
        double desiredPos = currentState.pos;
        return Math.abs(curPos - desiredPos) <= tolerance;
    }
}
