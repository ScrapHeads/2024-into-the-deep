package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.PLACE_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake.controlState.RESET_LIFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PICK_UP_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.PLACE_ROTATE;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.RESET_ROTATE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Commands.FollowDrivePath;
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

@Autonomous(name = "HighBasketBlue", group = "ScrapHeads")
public class HighBasketBlue extends CommandOpMode {
    //Creating all the variables used in the code
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

        // Might need to change pose2d for field centric reasons, will need to change for autos
//        drivetrain = new Drivetrain(hardwareMap, new Pose2d(33, 63, Math.toRadians(0)));
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
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

        TrajectoryActionBuilder firstTrajectory = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(23, 28.5, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder secondTrajectory = drivetrain.actionBuilder(new Pose2d(15, 28.5, Math.toRadians(-45)))
                .splineToLinearHeading(new Pose2d(27, 19.5, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder thirdTrajectory = drivetrain.actionBuilder(new Pose2d(27, 19, Math.toRadians(-45)))
                .splineToLinearHeading(new Pose2d(15, -15, Math.toRadians(0)), Math.toRadians(0));

        schedule(new SequentialCommandGroup(

                new ParallelCommandGroup(
                        new liftArmIntake(armLiftIntake, 1, PLACE_LIFT),
                        new RotateArmIntake(armRotateIntake, 1, PLACE_ROTATE),
                        new WaitCommand(1300)
                ),

                new ParallelCommandGroup(
                        new InstantCommand(() -> claw.setPower(1))
                ),

                new WaitCommand(2000),

                new ParallelCommandGroup(
                        new liftArmIntake(armLiftIntake, 1, RESET_LIFT)
                ),

                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new InstantCommand(() -> claw.setPower(0)),
                        new RotateArmIntake(armRotateIntake, 1, RESET_ROTATE)
                ),

                new ParallelCommandGroup(
                        new FollowDrivePath(drivetrain, firstTrajectory.build())
                ),

                new ParallelCommandGroup(
                        new RotateArmIntake(armRotateIntake, 1, PICK_UP_ROTATE),
                        new InstantCommand(() -> claw.setPower(-1)),
                        new FollowDrivePath(drivetrain, secondTrajectory.build())
                ),

                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new InstantCommand(() -> claw.setPower(0)),
                        new RotateArmIntake(armRotateIntake, 1, PLACE_ROTATE),
                        new FollowDrivePath(drivetrain, thirdTrajectory.build())
                )

        ));


    }
}
