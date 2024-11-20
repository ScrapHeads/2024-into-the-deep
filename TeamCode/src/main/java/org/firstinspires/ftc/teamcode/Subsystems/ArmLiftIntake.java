package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.function.Supplier;

public class ArmLiftIntake implements Subsystem {
    private static final double ticksToInches = -114.25;
    private static final double maxArmLengthIn = 38;

    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armLiftIntake;
    private Supplier<Rotation2d> rotSupplier;

    public ArmLiftIntake(Supplier<Rotation2d> rotSupplier) {
        //Linking armLift in the code to the motor on the robot
        armLiftIntake = new MotorEx(hm, "armLiftIntake", Motor.GoBILDA.RPM_312);
//        armLiftIntake.resetEncoder();
        this.rotSupplier = rotSupplier;

        //Setting the configuration for the motor
        armLiftIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ElevatorTicks", armLiftIntake.getCurrentPosition());
        packet.put("x", rotSupplier.get().getCos());
        dashboard.sendTelemetryPacket(packet);
    }

    public void setPower(double power) {
        //Setting the lift to the power in MainTeleop
        Rotation2d rot = rotSupplier.get();

        double currentExtension = Math.abs(armLiftIntake.get() / ticksToInches);
        double maxExtensionIn = (21 / Math.abs(rot.getCos())) - 15;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Max Extension", maxExtensionIn);
        packet.put("Current Extension",currentExtension);
        dashboard.sendTelemetryPacket(packet);

        if (currentExtension >= maxExtensionIn && power > 0) {
            armLiftIntake.set(0);
        } else {
            armLiftIntake.set(power);
        }
    }

}
