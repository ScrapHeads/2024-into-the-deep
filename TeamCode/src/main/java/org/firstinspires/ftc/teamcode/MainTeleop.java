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
import org.firstinspires.ftc.teamcode.Commands.moveClimber;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "MainTeleop", group = "ScrapHeads")
public class MainTeleop extends CommandOpMode {
    GamepadEx driver = null;
    Drivetrain drivetrain = null;
    Climber climber = null;

    @Override
    public void initialize() {
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        driver = new GamepadEx(gamepad1);

        // Might need to change pose2d for field centric reasons, will need to change for autos
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));
        drivetrain.register();

        climber = new Climber();
        climber.register();

        assignControls();
    }

    public void assignControls() {
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

        driver.getGamepadButton(DPAD_UP)
                .whenPressed(new moveClimber(climber, 0.3))
                .whenReleased(new moveClimber(climber, 0));
        driver.getGamepadButton(DPAD_DOWN)
                .whenPressed(new moveClimber(climber, -0.3))
                .whenReleased(new moveClimber(climber, 0));

    }


}
