package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.*;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.*;
import static org.firstinspires.ftc.teamcode.Subsystems.Climber.controlState.*;



import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Automation.HangEndGame;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHB;
import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Commands.liftClimber;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "MainTeleop", group = "ScrapHeads")
public class MainTeleop extends CommandOpMode {
    //Creating all the variables used in the code
    //Creating controller
    GamepadEx driver = null;

    //Creating drivetrain
    Drivetrain drivetrain = null;

    //Creating climber
    Climber climber = null;

    //Creating claw
    Claw claw = null;

    //Creating armLiftIntake
    ArmLiftIntake armLiftIntake = null;

    //creating armRotateIntake
    ArmRotateIntake armRotateIntake = null;

    //creating armLiftClipper
    ArmLiftClipper armLiftClipper = null;


    double currentTime = getRuntime();

    double totalDriverControlTime = 120;

    public enum PickUpStates {
        STATE_ONE,
        STATE_TWO,
        STATE_THREE
    }

    private PickUpStates currentPickUpState = PickUpStates.STATE_THREE;

    private boolean isSlowMode = false;

    @Override
    public void initialize() {
        //Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        //Initializing the controller 1 for inputs in assignControls
        driver = new GamepadEx(gamepad1);

        // Might need to change pose2d for field centric reasons, will need to change for autos
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));
        drivetrain.register();

        //Initializing the climber
        climber = new Climber();
        climber.register();

        //Initializing the claw
        claw = new Claw();
        claw.register();

        //Initializing the armRotateIntake
        armRotateIntake = new ArmRotateIntake();
        armRotateIntake.register();

        //Initializing the armLiftIntake
        armLiftIntake = new ArmLiftIntake(armRotateIntake::getRot);
        armLiftIntake.register();

        //Initializing the armLiftClipper
//        armLiftClipper = new ArmLiftClipper();
//        armLiftClipper.register();

        assignControls();
    }

    public void assignControls() {
        //Inputs for the drive train
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

        double timeLeft = totalDriverControlTime - currentTime;

//        if (timeLeft <= 30) {
//            new liftClimber(climber, 1, LIFT_HANG);
//        }

//        new Trigger(() -> isSlowMode)
//                .whileActiveOnce(new DriveContinous(drivetrain, driver, 0.5));

        //Inputs for the climber
        driver.getGamepadButton(DPAD_UP)
                .whenPressed(new liftClimber(climber, 1.0, MANUAL_HANG))
                .whenReleased(new liftClimber(climber, 0, STOP_HANG));
        driver.getGamepadButton(DPAD_DOWN)
                .whenPressed(new liftClimber(climber, -1.0, MANUAL_HANG))
                .whenReleased(new liftClimber(climber, 0, STOP_HANG));

        //Inputs for the whole arm for the intake side
        //Inputs for the armLiftIntake
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .whenActive(new liftArmIntake(armLiftIntake, .75, MANUAL_LIFT))
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
                .whenPressed(new PlacePieceHB(armLiftIntake, armRotateIntake, claw));

        driver.getGamepadButton(X)
                .whenPressed(new InstantCommand(this::advancedPickUpStates));

        new Trigger(() -> currentPickUpState == PickUpStates.STATE_ONE)
                .whenActive(
                        new RotateArmIntake(armRotateIntake, 1, PRE_PICK_UP_ROTATE)
                        );

        new Trigger(() -> currentPickUpState == PickUpStates.STATE_TWO)
                .whenActive(
                        new SequentialCommandGroup(
                                new RotateArmIntake(armRotateIntake, 1, PICK_UP_ROTATE),
                                new intakeClaw(claw, -1).andThen(
                                    new RotateArmIntake(armRotateIntake, 1, PRE_PICK_UP_ROTATE)
                                )
                        )
                );

        new Trigger(() -> currentPickUpState == PickUpStates.STATE_THREE)
                .whenActive(
                        new ParallelCommandGroup(
                                new RotateArmIntake(armRotateIntake, 1, TUCK_ROTATE),
                                new intakeClaw(claw, 0)
                        )
                );

        driver.getGamepadButton(START)
                .whenPressed(new HangEndGame(armLiftIntake, armRotateIntake, climber));

        //Inputs for the claw intake
        driver.getGamepadButton(B)
                .whenPressed(new intakeClaw(claw, 1))
                .whenReleased(new intakeClaw(claw, 0));
        driver.getGamepadButton(A)
                .whenPressed(new intakeClaw(claw, -1))
//                .whenPressed(new InstantCommand(() -> {isSlowMode = false;}))
                .whenReleased(new intakeClaw(claw, 0));

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
}
