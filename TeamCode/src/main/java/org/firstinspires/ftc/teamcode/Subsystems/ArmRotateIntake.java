package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

public class ArmRotateIntake implements Subsystem {
    private static final double gearRatio = 10.0;
    //private static final double ticksPerRadian = ((537.7 * gearRatio) / 2) * Math.PI;
    private static final double radiansPerTick = ((6.28 / 537.7) / gearRatio);

    private static final double feedForward = -0.1;

    //Designating the armLift variable to be set in the Arm function
    private final MotorEx armRotateIntake;

    private final PIDController pidController = new PIDController(0.05, 0, 0);

    private final ProfiledPIDController pickUpPidController = new ProfiledPIDController(0.1, 0.001, 0.002, new TrapezoidProfile.Constraints(50, 90));

    public enum controlState {
        PLACE_ROTATE(73),
        HB_AFTER(80),
        PICK_UP_ROTATE(0),
        MANUAL_ROTATE(-10),
        SWAP_STATES_ROTATE(-55),
        PRE_PICK_UP_ROTATE(35),
        OVER_WALL_ROTATE(65),
        TUCK_ROTATE(100),
        PRE_HANG_HIGH_ROTATE(105),
        HANG_HIGH_ROTATE(110),
        HOLD_ROTATE(15);

        public final double pos;
        controlState(double pos) {
            this.pos = pos;
        }
    }

    private controlState currentState = controlState.TUCK_ROTATE;

    private double manualPower = 0;
    private double savedPosition = 0;

    boolean whatState = true;

    double output;

    public ArmRotateIntake() {
        //Linking armLift in the code to the motor on the robot
        armRotateIntake = new MotorEx(hm, "armRotateIntake", Motor.GoBILDA.RPM_312);
        armRotateIntake.resetEncoder();

        //Setting the configuration for the motor
        armRotateIntake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        pidController.setTolerance(1);
        pidController.reset();
        pickUpPidController.setTolerance(1);
        pickUpPidController.reset();
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("degrees", getRot().getDegrees());
        packet.put("Arm pos", armRotateIntake.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);

        boolean isProfiled = false;

        switch (currentState) {
            case MANUAL_ROTATE:
                armRotateIntake.set(manualPower);
                return;
            case PICK_UP_ROTATE:
//                pickUpPidController.setSetPoint(controlState.PICK_UP_ROTATE.pos);
                isProfiled = true;
                pickUpPidController.setGoal(controlState.PICK_UP_ROTATE.pos);
                break;
            case PLACE_ROTATE:
                pidController.setSetPoint(controlState.PLACE_ROTATE.pos);
                break;
            case HB_AFTER:
                pidController.setSetPoint(controlState.HB_AFTER.pos);
                break;
            case HOLD_ROTATE:
                pidController.setSetPoint(savedPosition);
                break;
            case PRE_PICK_UP_ROTATE:
                isProfiled = true;
                pickUpPidController.setGoal(controlState.PRE_PICK_UP_ROTATE.pos);
                break;
            case OVER_WALL_ROTATE:
                isProfiled = true;
                pickUpPidController.setGoal(controlState.OVER_WALL_ROTATE.pos);
                break;
            case PRE_HANG_HIGH_ROTATE:
                pidController.setSetPoint(controlState.PRE_HANG_HIGH_ROTATE.pos);
                break;
            case HANG_HIGH_ROTATE:
                pidController.setSetPoint(controlState.HANG_HIGH_ROTATE.pos);
                break;


            case TUCK_ROTATE:
                pidController.setSetPoint(controlState.TUCK_ROTATE.pos);
        }

        double startingOffset = 2121;
        double currentDegrees = new Rotation2d((armRotateIntake.getCurrentPosition() + startingOffset) * radiansPerTick).getDegrees();

        output = isProfiled ? pickUpPidController.calculate(currentDegrees) + feedForward : pidController.calculate(currentDegrees);

        if ((isProfiled && pickUpPidController.atGoal()) || (!isProfiled && pidController.atSetPoint())) {
            armRotateIntake.set(0);
        } else {
            armRotateIntake.set(output);
        }
        TelemetryPacket random = new TelemetryPacket();
        random.put("Rotation output", output);
//        dashboard.sendTelemetryPacket(random);
    }

    public Rotation2d getRot() {
        double startingOffset = 2127;
        double rad = (armRotateIntake.getCurrentPosition() + startingOffset) * radiansPerTick;
        return new Rotation2d(rad);
    }

    public void setPower(double power, controlState state) {

        currentState = state;
        if (currentState == controlState.MANUAL_ROTATE) {
            manualPower = power;
        } else if (currentState == controlState.HOLD_ROTATE) {
            savedPosition = getRot().getDegrees();
        }
        else if (currentState == controlState.PLACE_ROTATE) {
            pidController.setSetPoint(controlState.PLACE_ROTATE.pos);
        } else if (currentState == controlState.HB_AFTER) {
            pidController.setSetPoint(controlState.HB_AFTER.pos);
        }
        else if (currentState == controlState.TUCK_ROTATE) {
            pidController.setSetPoint(controlState.TUCK_ROTATE.pos);
        } else if (currentState == controlState.PRE_PICK_UP_ROTATE) {
            pickUpPidController.setGoal(controlState.PRE_PICK_UP_ROTATE.pos);
        } else if (currentState == controlState.OVER_WALL_ROTATE) {
            pickUpPidController.setGoal(controlState.OVER_WALL_ROTATE.pos);
        }
        else if (currentState == controlState.HANG_HIGH_ROTATE) {
            pickUpPidController.setGoal(controlState.HANG_HIGH_ROTATE.pos);
        } else if (currentState == controlState.PRE_HANG_HIGH_ROTATE) {
            pickUpPidController.setGoal(controlState.PRE_HANG_HIGH_ROTATE.pos);
        }
        else if (currentState == controlState.PICK_UP_ROTATE) {
            pickUpPidController.setGoal(controlState.PICK_UP_ROTATE.pos);
        }

    }

    public boolean isAtPosition(double tolerance) {
        double curPos = getRot().getDegrees();
        double desiredPos = currentState.pos;
        return Math.abs(curPos - desiredPos) <= tolerance;
    }
}
