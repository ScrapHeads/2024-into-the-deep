package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.function.Supplier;

public class ArmLiftIntake implements Subsystem {
    private static final double ticksToInches = -114.25;

    private final PIDController pidController = new PIDController(0.3, 0, 0);

    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armLiftIntake;
    private Supplier<Rotation2d> rotSupplier;

    public enum controlState {
        PLACE_LIFT(33),
        PLACE_LIFT_EXTEND(36),
        PICK_UP_LIFT(3),
        RESET_LIFT(0),
        MANUAL_LIFT(-2),
        MANUAL_REVERSE(-3),
        SWAP_STATES_LIFT(-60),
        PRE_PLACE_AUTO(15),
        PRE_HANG_HIGH_LIFT(22.6),
        HANG_HIGH_LIFT(13.5),
        HOLD_LIFT(-1);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private ArmLiftIntake.controlState currentState = controlState.HOLD_LIFT;

    private double manualPower = 0;
    private double savedPosition = 0;

    boolean whatState = true;


    public ArmLiftIntake(Supplier<Rotation2d> rotSupplier) {
        //Linking armLift in the code to the motor on the robot
        armLiftIntake = new MotorEx(hm, "armLiftIntake", Motor.GoBILDA.RPM_312);

        this.rotSupplier = rotSupplier;
        armLiftIntake.resetEncoder();

        //Setting the configuration for the motor
        armLiftIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        pidController.setTolerance(.25);
    }

    @Override
    public void periodic() {
        //setPower(armLiftIntake.get());
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ElevatorTicks", armLiftIntake.getCurrentPosition());
        packet.put("Cos of Rot", rotSupplier.get().getCos());
//        dashboard.sendTelemetryPacket(packet);

        double maxExtensionIn = getMaxExtensionIn();

        switch (currentState) {
            case MANUAL_REVERSE:
                setPower(manualPower, controlState.MANUAL_REVERSE);
                return;
            case MANUAL_LIFT:
                pidController.setSetPoint(maxExtensionIn);
                break;
//                setPower(manualPower, controlState.MANUAL_LIFT);
//                return;
            case PICK_UP_LIFT:
                pidController.setSetPoint(controlState.PICK_UP_LIFT.pos);
                break;
            case PLACE_LIFT:
                pidController.setSetPoint(controlState.PLACE_LIFT.pos);
                break;
            case PLACE_LIFT_EXTEND:
                pidController.setSetPoint(controlState.PLACE_LIFT_EXTEND.pos);
                break;
            case PRE_PLACE_AUTO:
                pidController.setSetPoint(controlState.PRE_PLACE_AUTO.pos);
                break;
            case PRE_HANG_HIGH_LIFT:
                pidController.setSetPoint(controlState.PRE_HANG_HIGH_LIFT.pos);
                break;
            case HANG_HIGH_LIFT:
                pidController.setSetPoint(controlState.HANG_HIGH_LIFT.pos);
                break;
            case HOLD_LIFT:
                if (savedPosition > maxExtensionIn) {
                    savedPosition = maxExtensionIn;
                }
                pidController.setSetPoint(savedPosition);
                break;
            case RESET_LIFT:
                pidController.setSetPoint(controlState.RESET_LIFT.pos);
                break;
        }

        double currentExtension = Math.abs(armLiftIntake.getCurrentPosition() / ticksToInches);
        double output = -pidController.calculate(currentExtension);

        if (pidController.atSetPoint()) {
            armLiftIntake.set(0);
        } else {
            armLiftIntake.set(output);
        }

        TelemetryPacket random = new TelemetryPacket();
        random.put("lift output", output);
        packet.put("Max Extension", maxExtensionIn);
        packet.put("Current Extension", currentExtension);
        random.put("Current State", currentState);
        dashboard.sendTelemetryPacket(random);
    }

    public void checkState() {
        if (whatState) {
            whatState = false;
            setPower(1, controlState.PLACE_LIFT);
        } else {
            whatState = true;
            setPower(1, controlState.RESET_LIFT);
        }
    }

    public void setPower(double power, controlState state) {
        //Setting the lift to the power in MainTeleop
        if (state == controlState.SWAP_STATES_LIFT) {
            checkState();
        }

        double currentExtension = Math.abs(armLiftIntake.getCurrentPosition() / ticksToInches);

        double maxExtensionIn = getMaxExtensionIn();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Max Extension", maxExtensionIn);
        packet.put("Current Extension",currentExtension);
        packet.put("Tick lift", armLiftIntake.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);

        currentState = state;

        if (currentExtension < maxExtensionIn && power < 0 && currentState == controlState.MANUAL_LIFT) {
            pidController.setSetPoint(maxExtensionIn);
//            armLiftIntake.set(power);
//            manualPower = power;
        }  else if (power > 0 && currentState == controlState.MANUAL_REVERSE) {
            armLiftIntake.set(power);
            manualPower = power;
        }
        else if (currentState == controlState.PLACE_LIFT) {
            pidController.setSetPoint(controlState.PLACE_LIFT.pos);
        }else if (currentState == controlState.PLACE_LIFT_EXTEND) {
            pidController.setSetPoint(controlState.PLACE_LIFT_EXTEND.pos);
        } else if (currentState == controlState.RESET_LIFT) {
            pidController.setSetPoint(controlState.RESET_LIFT.pos);
        } else if (currentState == controlState.PRE_PLACE_AUTO) {
            pidController.setSetPoint(controlState.PRE_PLACE_AUTO.pos);
        }
        else if (currentState == controlState.PRE_HANG_HIGH_LIFT) {
            pidController.setSetPoint(controlState.PRE_HANG_HIGH_LIFT.pos);
        } else if (currentState == controlState.HANG_HIGH_LIFT) {
            pidController.setSetPoint(controlState.HANG_HIGH_LIFT.pos);
        }
        else { // power is 0
            armLiftIntake.set(0);
            savedPosition = currentExtension;
            currentState = controlState.HOLD_LIFT;
        }

//        if (currentExtension >= maxExtensionIn && power < 0) {
//            armLiftIntake.set(0);
//        } else {
//            armLiftIntake.set(power);
//        }
    }

    private double getMaxExtensionIn() {
        double maxExt = 0;
        double capExt = 33;

        if (rotSupplier.get().getDegrees() <= 90) {
            maxExt = (21 / Math.abs(rotSupplier.get().getCos())) - 17;
        } else if (rotSupplier.get().getDegrees() > 90) {
            maxExt = (21 / Math.abs(rotSupplier.get().getCos())) - 19;
        }
        if (maxExt > capExt) {
            maxExt = capExt;
        }
        return maxExt;
    }

    public boolean isAtPosition(double tolerance) {
        double curPos = Math.abs(armLiftIntake.getCurrentPosition() / ticksToInches);
        double desiredPos = currentState.pos;
        return Math.abs(curPos - desiredPos) <= tolerance;
    }

}
