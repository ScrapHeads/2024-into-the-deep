package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.liftArm;
import org.firstinspires.ftc.teamcode.Commands.liftClimber;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
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
    //Creating arm
    Arm arm = null;

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

        //Initializing the arm
        arm = new Arm();
        arm.register();

        assignControls();
    }

    public void assignControls() {
        //Inputs for the drive train
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

        //Inputs for the climber
        driver.getGamepadButton(DPAD_UP)
                .whenPressed(new liftClimber(climber, 0.3))
                .whenReleased(new liftClimber(climber, 0));
        driver.getGamepadButton(DPAD_DOWN)
                .whenPressed(new liftClimber(climber, -0.3))
                .whenReleased(new liftClimber(climber, 0));

        //Inputs for the arm
        driver.getGamepadButton(RIGHT_BUMPER)
                .whenPressed(new liftArm(arm, 0.3))
                .whenReleased(new liftArm(arm, 0));
        driver.getGamepadButton(LEFT_BUMPER)
                .whenPressed(new liftArm(arm, -0.3))
                .whenReleased(new liftArm(arm, 0));

        //Inputs for the claw
//        driver.getGamepadButton(Tri)
//                .whenPressed(new intakeClaw(claw, .3))
//                .whenReleased(new intakeClaw(claw, 0));
    }


}
