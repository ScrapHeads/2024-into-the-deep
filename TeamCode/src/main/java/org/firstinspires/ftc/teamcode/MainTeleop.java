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
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeClaw;
import org.firstinspires.ftc.teamcode.Commands.liftArmIntake;
import org.firstinspires.ftc.teamcode.Commands.liftClimber;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateClipper;
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

    //creating armRotateClipper
    ArmRotateClipper armRotateClipper = null;

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

        //Initializing the armRotateClipper
//        armRotateClipper = new ArmRotateClipper();
//        armRotateClipper.register();



        assignControls();
    }

    public void assignControls() {
        //Inputs for the drive train
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

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
                .whenPressed(new liftArmIntake(armLiftIntake, 1, PLACE_LIFT))
                .whenPressed(new RotateArmIntake(armRotateIntake, 1, PLACE_ROTATE));

        driver.getGamepadButton(X)
                .whenPressed(new RotateArmIntake(armRotateIntake, 1 , PICK_UP_ROTATE));

        driver.getGamepadButton(START)
                .whenPressed(new liftClimber(climber, 1, LIFT_HANG));

        //Inputs for the claw intake
        driver.getGamepadButton(B)
                .whenPressed(new intakeClaw(claw, 1))
                .whenReleased(new intakeClaw(claw, 0));
        driver.getGamepadButton(A)
                .whenPressed(new intakeClaw(claw, -1))
                .whenReleased(new intakeClaw(claw, 0));

//        driver.getGamepadButton(Y)
//                .whenPressed(new RotateArmIntake())


        //Trigger example don't uncomment
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
//                .whenActive(new intakeClaw(claw, 1))
//                .whenInactive(new intakeClaw(claw, 0));
//
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
//                .whenActive(new intakeClaw(claw, -1))
//                .whenInactive(new intakeClaw(claw, 0));
    }


}
