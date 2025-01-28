package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.closedClaw;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.openClaw;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper.controlState.*;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.*;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.*;
import static org.firstinspires.ftc.teamcode.Subsystems.Climber.controlState.*;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Automation.HangEndGame;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBTele;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlaceSampleZone;
import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.RotateClipperClaw;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmClipper;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Commands.liftClimber;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.ClipperClaw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "MainTeleopClipper", group = "ScrapHeads")
public class MainTeleopClipper extends CommandOpMode {
    //Creating all the variables used in the code

    //Creating controller
    GamepadEx driver = null;
    GamepadEx driver2 = null;

    //Creating drivetrain
    Drivetrain drivetrain = null;

    //Creating climber
    Climber climber = null;

    //Creating claw
    Claw claw = null;

    //Creating claw
    ClipperClaw clipperClaw = null;

    //Creating armLiftIntake
    ArmLiftIntake armLiftIntake = null;

    //creating armRotateIntake
    ArmRotateIntake armRotateIntake = null;

    //creating armLiftClipper
    ArmLiftClipper armLiftClipper = null;

    //Set time to lift Climber
    double timeTillClimb = 100;

    private boolean whatTime;

    ElapsedTime timePassed = new ElapsedTime();

    public enum PickUpStates {
        STATE_ONE,
        STATE_TWO,
        STATE_THREE
    }

    public enum ClipperStates {
        STATE_ONE,
        STATE_TWO
    }

    public enum controllerStates {
        CONTROLLER_ONE,
        CONTROLLER_TWO
    }

    private PickUpStates currentPickUpState = PickUpStates.STATE_THREE;

    private ClipperStates currentClipperStates = ClipperStates.STATE_ONE;

    private controllerStates currentController = controllerStates.CONTROLLER_TWO;

    private boolean isSlowMode = false;

    @Override
    public void initialize() {
        //Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        //Initializing the controller 1 for inputs in assignControls
        driver = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Might need to change pose2d for field centric reasons, will need to change for autos
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));
        drivetrain.register();

        //Initializing the climber
        climber = new Climber();
        climber.register();

        //Initializing the claw
        claw = new Claw();
        claw.register();

        //Initializing the claw clipper
        clipperClaw = new ClipperClaw();
        clipperClaw.register();

        //Initializing the armRotateIntake
        armRotateIntake = new ArmRotateIntake();
        armRotateIntake.register();

        //Initializing the armLiftIntake
        armLiftIntake = new ArmLiftIntake(armRotateIntake::getRot);
        armLiftIntake.register();

        //Initializing the armLiftClipper
        armLiftClipper = new ArmLiftClipper();
        armLiftClipper.register();

        timePassed.reset();
        assignControls();
    }

    public void assignControls() {
        //Inputs for the drive train

        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver2, 1));

        driver.getGamepadButton(START)
                .whenPressed(new InstantCommand(() -> currentController = controllerStates.CONTROLLER_ONE));

        driver2.getGamepadButton(START)
                .whenPressed(new InstantCommand(() -> currentController = controllerStates.CONTROLLER_TWO));

        new Trigger(() -> currentController == controllerStates.CONTROLLER_ONE)
                .whenActive(
                        new DriveContinous(drivetrain, driver, 1)
                );

        new Trigger(() -> currentController == controllerStates.CONTROLLER_TWO)
                .whenActive(
                        new DriveContinous(drivetrain, driver2, 1)
                );

        //Statements for in game functions
        double timeSeconds = timePassed.seconds();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Time Passed", timeSeconds);
        dashboard.sendTelemetryPacket(packet);

        new Trigger(() -> timePassed.seconds() >= timeTillClimb)
                .whileActiveOnce(new liftClimber(climber, 1, HANG_ONE));

        new Trigger(() -> isSlowMode)
                .whileActiveOnce(new DriveContinous(drivetrain, driver, 0.3));

        new Trigger(() -> !isSlowMode)
                .whileActiveOnce(new DriveContinous(drivetrain, driver, 1.0));

        //Statements for in game functions controller one

        //Inputs for the climber
        driver.getGamepadButton(DPAD_UP)
                .whenPressed(new liftClimber(climber, 1.0, MANUAL_HANG))
                .whenReleased(new liftClimber(climber, 0, STOP_HANG));
        driver.getGamepadButton(DPAD_DOWN)
                .whenPressed(new liftClimber(climber, -1.0, MANUAL_HANG))
                .whenReleased(new liftClimber(climber, 0, STOP_HANG));

        //Inputs for the armLiftIntake
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .whenActive(new liftArmIntake(armLiftIntake, .75, MANUAL_REVERSE))
                .whenInactive(new liftArmIntake(armLiftIntake, 0, HOLD_LIFT));

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whenActive(new liftArmIntake(armLiftIntake, -.75, MANUAL_LIFT))
                .whenInactive(new liftArmIntake(armLiftIntake, 0, HOLD_LIFT));

        //Inputs for the armRotateIntake
        driver.getGamepadButton(DPAD_LEFT)
                .whenPressed(new RotateArmIntake(armRotateIntake, 0.4, MANUAL_ROTATE))
                .whenReleased(new RotateArmIntake(armRotateIntake, 0, HOLD_ROTATE));
        driver.getGamepadButton(DPAD_RIGHT)
                .whenPressed(new RotateArmIntake(armRotateIntake, -0.4, MANUAL_ROTATE))
                .whenReleased(new RotateArmIntake(armRotateIntake, 0, HOLD_ROTATE));

        //Pid controls
        driver.getGamepadButton(Y)
                .whenPressed(
                        new ParallelCommandGroup(
                                new PlacePieceHBTele(armLiftIntake, armRotateIntake, claw),
                                new InstantCommand(() -> {isSlowMode = true;})
                        ).whenFinished(() -> {isSlowMode = false;})
                );

        driver.getGamepadButton(X)
                .whenPressed(new InstantCommand(this::advancedPickUpStates));

        new Trigger(() -> currentPickUpState == PickUpStates.STATE_ONE)
                .whenActive(
                        new RotateArmIntake(armRotateIntake, 1, PRE_PICK_UP_ROTATE)
                )
                .whileActiveOnce(new InstantCommand(() -> {isSlowMode = true;}));

        new Trigger(() -> currentPickUpState == PickUpStates.STATE_TWO)
                .whenActive(
                        new SequentialCommandGroup(
                                new RotateArmIntake(armRotateIntake, 1, PICK_UP_ROTATE),
                                new intakeClaw(claw, -1).andThen(
                                        new liftArmIntake(armLiftIntake, 1, RESET_LIFT),
                                        new RotateArmIntake(armRotateIntake, 1, PRE_PICK_UP_ROTATE)
                                )
                        )
                );

        new Trigger(() -> currentPickUpState == PickUpStates.STATE_THREE)
                .whileActiveOnce(new InstantCommand(() -> {isSlowMode = false;}))
                .whenActive(
                        new ParallelCommandGroup(
                                new RotateArmIntake(armRotateIntake, 1, TUCK_ROTATE),
                                new intakeClaw(claw, 0),
                                new DriveContinous(drivetrain, driver, 1)
                        )
                );

        driver.getGamepadButton(BACK)
                .whenPressed(new HangEndGame(armLiftIntake, armRotateIntake, climber));

        //Inputs for the claw intake
        driver.getGamepadButton(B)
                .whenPressed(new intakeClaw(claw, 1))
                .whenReleased(new intakeClaw(claw, 0));
        driver.getGamepadButton(A)
                .whenPressed(new intakeClaw(claw, -1))
                .whenReleased(new intakeClaw(claw, 0));


        //Statements for in game functions controller TWO

        //Inputs for the climber
        driver2.getGamepadButton(DPAD_UP)
                .whenPressed(new liftClimber(climber, 1.0, MANUAL_HANG))
                .whenReleased(new liftClimber(climber, 0, STOP_HANG));
        driver2.getGamepadButton(DPAD_DOWN)
                .whenPressed(new liftClimber(climber, -1.0, MANUAL_HANG))
                .whenReleased(new liftClimber(climber, 0, STOP_HANG));

        //Inputs for the liftArmClipper
        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .whenActive(new liftArmClipper(armLiftClipper, 1, MANUAL_CLIPPER))
                .whenInactive(new liftArmClipper(armLiftClipper, 0, HOLD_CLIPPER));

        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whenActive(new liftArmClipper(armLiftClipper, -1, MANUAL_CLIPPER))
                .whenInactive(new liftArmClipper(armLiftClipper, 0, HOLD_CLIPPER));

        //Inputs for the claw clipper
        driver2.getGamepadButton(A)
                .whenPressed(new RotateClipperClaw(clipperClaw, closedClaw));
        driver2.getGamepadButton(B)
                .whenPressed(new RotateClipperClaw(clipperClaw, openClaw));

        //Pid controls

        driver2.getGamepadButton(Y)
                .whenPressed(new InstantCommand(this::advancedClipperStates));

        new Trigger(() -> currentClipperStates == ClipperStates.STATE_ONE)
                .whenActive(
                        new ParallelCommandGroup(
                                new liftArmClipper(armLiftClipper, 1, PICK_UP_CLIPPER),
                                new WaitUntilCommand(() -> armLiftClipper.isAtPosition(16.5)).andThen(
                                        new RotateClipperClaw(clipperClaw, openClaw))
                        )
                );

        new Trigger(() -> currentClipperStates == ClipperStates.STATE_TWO)
                .whenActive(
                        new ParallelCommandGroup(
                                new RotateClipperClaw(clipperClaw, closedClaw),
                                new liftArmClipper(armLiftClipper, 1, PLACE_CLIPPER)
                        )
                );

        driver2.getGamepadButton(BACK)
                .whenPressed(new HangEndGame(armLiftIntake, armRotateIntake, climber));

        driver2.getGamepadButton(RIGHT_BUMPER)
                .whenPressed(new PlaceSampleZone(armRotateIntake, claw));

        //Trigger example don't uncomment
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
//                .whenActive(new intakeClaw(claw, 1))
//                .whenInactive(new intakeClaw(claw, 0));
//
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
//                .whenActive(new intakeClaw(claw, -1))
//                .whenInactive(new intakeClaw(claw, 0));
    }

    private void advancedPickUpStates() {
        switch(currentPickUpState) {
            case STATE_ONE:
                currentPickUpState = PickUpStates.STATE_TWO;
                break;
            case STATE_TWO:
                currentPickUpState = PickUpStates.STATE_THREE;
                break;
            case STATE_THREE:
                currentPickUpState = PickUpStates.STATE_ONE;
                break;
        };
    }

    private void advancedClipperStates() {
        switch(currentClipperStates) {
            case STATE_ONE:
                currentClipperStates = ClipperStates.STATE_TWO;
                break;
            case STATE_TWO:
                currentClipperStates = ClipperStates.STATE_ONE;
                break;
        };
    }
}
