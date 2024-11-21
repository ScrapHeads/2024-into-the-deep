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
    private static final double maxArmLengthIn = 38;

    private final PIDController pidController = new PIDController(0.3, 0, 0);

    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armLiftIntake;
    private Supplier<Rotation2d> rotSupplier;

    public enum controlState {
        PLACE(90),
        PICK_UP(20),
        MANUAL(0),
        HOLD(15);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private ArmLiftIntake.controlState currentState = ArmLiftIntake.controlState.MANUAL;

    private double manualPower = 0;
    private double savedPosition = 0;

    public ArmLiftIntake(Supplier<Rotation2d> rotSupplier) {
        //Linking armLift in the code to the motor on the robot
        armLiftIntake = new MotorEx(hm, "armLiftIntake", Motor.GoBILDA.RPM_312);
//        armLiftIntake.resetEncoder();
        this.rotSupplier = rotSupplier;
        armLiftIntake.resetEncoder();

        //Setting the configuration for the motor
        armLiftIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pidController.setTolerance(.5);
    }

    @Override
    public void periodic() {
        //setPower(armLiftIntake.get());
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ElevatorTicks", armLiftIntake.getCurrentPosition());
        packet.put("Cos of Rot", rotSupplier.get().getCos());
        dashboard.sendTelemetryPacket(packet);

        double maxExtensionIn = getMaxExtensionIn();

        switch (currentState) {
            case MANUAL:
                setPower(manualPower);
                return;
            case PICK_UP:
                pidController.setSetPoint(ArmLiftIntake.controlState.PICK_UP.pos);
                break;
            case PLACE:
                pidController.setSetPoint(ArmLiftIntake.controlState.PLACE.pos);
                break;
            case HOLD:
                if (savedPosition > maxExtensionIn) {
                    savedPosition = maxExtensionIn;
                }
                pidController.setSetPoint(savedPosition);
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
        packet.put("Current State", currentState);
        dashboard.sendTelemetryPacket(random);

    }

    public void setPower(double power) {
        //Setting the lift to the power in MainTeleop

        double currentExtension = Math.abs(armLiftIntake.getCurrentPosition() / ticksToInches);

        double maxExtensionIn = getMaxExtensionIn();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Max Extension", maxExtensionIn);
        packet.put("Current Extension",currentExtension);
        packet.put("Tick lift", armLiftIntake.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);

        if (currentExtension < maxExtensionIn && power < 0) {
            armLiftIntake.set(power);
            currentState = controlState.MANUAL;
            manualPower = power;
        }  else if (power > 0) {
            armLiftIntake.set(power);
            currentState = controlState.MANUAL;
            manualPower = power;
        } else { // power is 0
            //armLiftIntake.set(0);
            savedPosition = currentExtension;
            currentState = controlState.HOLD;
        }

//        if (currentExtension >= maxExtensionIn && power < 0) {
//            armLiftIntake.set(0);
//        } else {
//            armLiftIntake.set(power);
//        }
    }



    private double getMaxExtensionIn() {
        double capExt = 33;
        double maxExt = 0;
        if (rotSupplier.get().getDegrees() < 90) {
            maxExt = (21 / Math.abs(rotSupplier.get().getCos())) - 15;
        } else if (rotSupplier.get().getDegrees() > 90) {
            maxExt = (21 / Math.abs(rotSupplier.get().getCos())) - 19;
        }
        if (maxExt > capExt) {
            maxExt = capExt;
        }
        return maxExt;
    }

}
