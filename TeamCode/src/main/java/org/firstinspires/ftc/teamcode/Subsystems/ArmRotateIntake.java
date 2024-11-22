package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmRotateIntake implements Subsystem {
    private static final double gearRatio = 10.0;
    //private static final double ticksPerRadian = ((537.7 * gearRatio) / 2) * Math.PI;
    private static final double ticksPerRadian = ((6.28 / 537.7) / gearRatio);

    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armRotateIntake;
    private final PIDController pidController = new PIDController(0.05, 0, 0);

    public enum controlState {
        PLACE_ROTATE(73),
        PICK_UP_ROTATE(1),
        MANUAL_ROTATE(0),
        SWAP_STATES_ROTATE(-55),
        RESET_ROTATE(45),
        HOLD_ROTATE(15);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private controlState currentState = controlState.MANUAL_ROTATE;

    private double manualPower = 0;
    private double savedPosition = 0;

    boolean whatState = true;

    public ArmRotateIntake() {
        //Linking armLift in the code to the motor on the robot
        armRotateIntake = new MotorEx(hm, "armRotateIntake", Motor.GoBILDA.RPM_312);
        armRotateIntake.resetEncoder();

        //Setting the configuration for the motor
        armRotateIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        pidController.setTolerance(1);
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("degrees", getRot().getDegrees());
        packet.put("Arm pos", armRotateIntake.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);

        switch (currentState) {
            case MANUAL_ROTATE:
                armRotateIntake.set(manualPower);
                return;
            case PICK_UP_ROTATE:
                pidController.setSetPoint(controlState.PICK_UP_ROTATE.pos);
                break;
            case PLACE_ROTATE:
                pidController.setSetPoint(controlState.PLACE_ROTATE.pos);
                break;
            case HOLD_ROTATE:
                pidController.setSetPoint(savedPosition);
                break;
            case RESET_ROTATE:
                pidController.setSetPoint(controlState.RESET_ROTATE.pos);
                break;
        }

        double startingOffset = 2127;
        double currentDegrees = new Rotation2d((armRotateIntake.getCurrentPosition() + startingOffset) * ticksPerRadian).getDegrees();
        double output = pidController.calculate(currentDegrees);

        if (pidController.atSetPoint()) {
            armRotateIntake.set(0);
        } else {
            armRotateIntake.set(output);
        }
        TelemetryPacket random = new TelemetryPacket();
        random.put("Rotation output", output);
        dashboard.sendTelemetryPacket(random);
    }

    public Rotation2d getRot() {
        double startingOffset = 2127;
        double rad = (armRotateIntake.getCurrentPosition() + startingOffset) * ticksPerRadian;
        return new Rotation2d(rad);
    }

    public void checkState() {
        if (whatState) {
            whatState = false;
            setPower(1, controlState.PLACE_ROTATE);
        } else {
            whatState = true;
            setPower(1, controlState.RESET_ROTATE);
        }
    }

    public void setPower(double power, controlState state) {
        //Setting the lift to the power in MainTeleop
//        currentState = controlState.MANUAL;
//        manualPower = power;
//        armRotateIntake.set(power);

        if (state == ArmRotateIntake.controlState.SWAP_STATES_ROTATE) {
            checkState();
        }

        currentState = state;
        if (currentState == controlState.MANUAL_ROTATE) {
            manualPower = power;
        } else if (currentState == controlState.HOLD_ROTATE) {
            savedPosition = getRot().getDegrees();
        } else if (currentState == controlState.PLACE_ROTATE) {
            pidController.setSetPoint(controlState.PLACE_ROTATE.pos);
        } else if (currentState == controlState.RESET_ROTATE) {
            pidController.setSetPoint(controlState.RESET_ROTATE.pos);
        } else if (currentState == controlState.PICK_UP_ROTATE) {
            pidController.setSetPoint(controlState.PICK_UP_ROTATE.pos);
        }

//        if (power != 0) {
//            //Setting the lift to the power in MainTeleop
//            currentState = controlState.MANUAL_ROTATE;
//            manualPower = power;
//        } else { //stay where you are
//            currentState = controlState.HOLD_ROTATE;
//            savedPosition = getRot().getDegrees();
//        }
    }
}
