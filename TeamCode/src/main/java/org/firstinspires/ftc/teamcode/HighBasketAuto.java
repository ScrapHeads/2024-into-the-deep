package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.TUCK_ROTATE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Automation.PickUpFloorAuto;
import org.firstinspires.ftc.teamcode.Commands.Automation.PickUpFloorAutoSecondSpikeHB;
import org.firstinspires.ftc.teamcode.Commands.Automation.PickUpFloorAutoThirdSpikeHB;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHB;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBAutoEnd;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBTele;
import org.firstinspires.ftc.teamcode.Commands.Automation.PrePlaceHBAuto;
import org.firstinspires.ftc.teamcode.Commands.FollowDrivePath;
import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import java.util.Arrays;

@Autonomous(name = "HighBasketAuto", group = "ScrapHeads")
public class HighBasketAuto extends CommandOpMode {
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

        TurnConstraints turnConstraints = new TurnConstraints(Math.PI, -Math.PI, Math.PI);
        VelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(40),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraint = new ProfileAccelConstraint(-25, 40);

        TrajectoryActionBuilder pickUpPreFirstBlock = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(15, 35.5, Math.toRadians(-45)), Math.toRadians(-45));

        TrajectoryActionBuilder pickUpFirstBlock = drivetrain.actionBuilder(new Pose2d(16, 34.5, Math.toRadians(-45)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(27, 28, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder placeFirstBlock = drivetrain.actionBuilder(new Pose2d(26, 27, Math.toRadians(-45)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(41, -4, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder pickUpPreSecondBlock = drivetrain.actionBuilder(new Pose2d(41, -4, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(23, 28, Math.toRadians(-45)), Math.toRadians(90));

        TrajectoryActionBuilder pickUpSecondBlock = drivetrain.actionBuilder(new Pose2d(25, 28, Math.toRadians(-45)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(38, 23, Math.toRadians(-45)), Math.toRadians(0));

        TrajectoryActionBuilder placeSecondBlock = drivetrain.actionBuilder(new Pose2d(33, 23, Math.toRadians(-45)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(39, -1, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder pickUpPreThirdBlock = drivetrain.actionBuilder(new Pose2d(35, -8, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(25, 15, Math.toRadians(0)), Math.toRadians(90));

        TrajectoryActionBuilder pickUpThirdBlock = drivetrain.actionBuilder(new Pose2d(16, 17, Math.toRadians(0)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(28, 15, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder placeThirdBlock = drivetrain.actionBuilder(new Pose2d(16, 17, Math.toRadians(0)), turnConstraints, velConstraint, accelConstraint)
                .splineToLinearHeading(new Pose2d(50.5, 0, Math.toRadians(45)), Math.toRadians(0));

        schedule(new SequentialCommandGroup(

                new RotateArmIntake(armRotateIntake, 1, TUCK_ROTATE),
                new PlacePieceHB(armLiftIntake, armRotateIntake, claw),

                new ParallelCommandGroup(
                        new PickUpFloorAuto(armRotateIntake),
                        new FollowDrivePath(drivetrain, pickUpPreFirstBlock.build())
                ),

                new ParallelCommandGroup(
                        new InstantCommand(() -> claw.setPower(-1)),
                        new FollowDrivePath(drivetrain, pickUpFirstBlock.build())
                ),

                new WaitCommand(250),

                new ParallelCommandGroup(
                        new InstantCommand(() -> claw.setPower(0)),
                        new PrePlaceHBAuto(armLiftIntake, armRotateIntake),
                        new FollowDrivePath(drivetrain, placeFirstBlock.build())
                ),
//
                new PlacePieceHBTele(armLiftIntake, armRotateIntake, claw),
//
                new ParallelCommandGroup(
                        new FollowDrivePath(drivetrain, pickUpPreSecondBlock.build()),
                        new PickUpFloorAutoSecondSpikeHB(armRotateIntake)
                        ),

                new ParallelCommandGroup(
                        new FollowDrivePath(drivetrain, pickUpSecondBlock.build()),
                        new InstantCommand(() -> claw.setPower(-1))
                ),

                new WaitCommand(250),
//
                new ParallelCommandGroup(
                        new InstantCommand(() -> claw.setPower(0)),
                        new PrePlaceHBAuto(armLiftIntake, armRotateIntake),
                        new FollowDrivePath(drivetrain, placeSecondBlock.build())
                ),

                new PlacePieceHBAutoEnd(armLiftIntake, armRotateIntake, claw),

                new ParallelCommandGroup(
                        new PickUpFloorAutoThirdSpikeHB(armRotateIntake),
                        new FollowDrivePath(drivetrain, pickUpPreThirdBlock.build())
                ),

                new ParallelCommandGroup(
                        new InstantCommand(() -> claw.setPower(-1)),
                        new FollowDrivePath(drivetrain, pickUpThirdBlock.build())
                ),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        new InstantCommand(() -> claw.setPower(0)),
                        new PrePlaceHBAuto(armLiftIntake, armRotateIntake),
                        new FollowDrivePath(drivetrain, placeThirdBlock.build())
                ),
//
                new PlacePieceHBAutoEnd(armLiftIntake, armRotateIntake, claw)

                ));


    }
}
