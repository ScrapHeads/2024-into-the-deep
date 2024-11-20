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
    private final PIDController pidController = new PIDController(0, 0, 0);

    public enum controlState {
        PLACE(90),
        PICK_UP(10),
        MANUAL(0);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private controlState currentState = controlState.MANUAL;

    private double manualPower = 0;

    public ArmRotateIntake() {
        //Linking armLift in the code to the motor on the robot
        armRotateIntake = new MotorEx(hm, "armRotateIntake", Motor.GoBILDA.RPM_312);
//        armRotateIntake.resetEncoder();

        //Setting the configuration for the motor
        armRotateIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        pidController.setTolerance(2);
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("degrees", getRot().getDegrees());
        packet.put("Arm pos", armRotateIntake.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
        // add telemetry
        switch (currentState) {
            case MANUAL:
                armRotateIntake.set(manualPower);
                return;
            case PICK_UP:
                pidController.setSetPoint(controlState.PICK_UP.pos);
                break;
            case PLACE:
                pidController.setSetPoint(controlState.PLACE.pos);
                break;
        }

        double currentDegrees = new Rotation2d(armRotateIntake.getCurrentPosition() * ticksPerRadian).getDegrees();
        double output = pidController.calculate(currentDegrees);

        if (pidController.atSetPoint()) {
            armRotateIntake.set(0);
        } else {
            armRotateIntake.set(output);
        }
    }

    public Rotation2d getRot() {
        double rad = armRotateIntake.getCurrentPosition() * ticksPerRadian;
        return new Rotation2d(rad);
    }


    public void setPosition(controlState state) {
        currentState = state;
    }

    public void setPower(double power) {
        //Setting the lift to the power in MainTeleop
        currentState = controlState.MANUAL;
        manualPower = power;
//        armRotateIntake.set(power);
    }
}
