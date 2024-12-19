package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake.controlState.TUCK_ROTATE;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper.controlState.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Automation.PickUpFloorAuto;
import org.firstinspires.ftc.teamcode.Commands.Automation.PickUpFloorAutoSecondSpikeHB;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHB;
import org.firstinspires.ftc.teamcode.Commands.Automation.PlacePieceHBTele;
import org.firstinspires.ftc.teamcode.Commands.Automation.PrePlaceHBAuto;
import org.firstinspires.ftc.teamcode.Commands.FollowDrivePath;
import org.firstinspires.ftc.teamcode.Commands.RotateArmIntake;
import org.firstinspires.ftc.teamcode.Commands.liftArmClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmLiftIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateClipper;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotateIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import java.util.Arrays;

@Autonomous(name = "ClippingAuto", group = "ScrapHeads")
public class ClippingAuto extends CommandOpMode {
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
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
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
        armLiftClipper = new ArmLiftClipper();
        armLiftClipper.register();

        //Initializing the armRotateClipper
//        armRotateClipper = new ArmRotateClipper();
//        armRotateClipper.register();

        TurnConstraints turnConstraints = new TurnConstraints(Math.PI, -Math.PI, Math.PI);
        VelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(40),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraint = new ProfileAccelConstraint(-25, 40);

        TrajectoryActionBuilder placeFirstClip = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(27, -5, Math.toRadians(180)), Math.toRadians(0));

        TrajectoryActionBuilder setUpPush = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder prePushFirstBlock = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder PushFirstBlock = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder prePushSecondBlock = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder pushSecondBlock = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder prePushThirdBlock = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder pushThirdBlock = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder setUpClipping = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder pickUpSecondClip = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder placeSecondClip = drivetrain.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0));

        schedule(new SequentialCommandGroup(

                new ParallelCommandGroup(
                        new FollowDrivePath(drivetrain, placeFirstClip.build()),
                        new liftArmClipper(armLiftClipper, 1, PLACE_CLIPPER)
                )

                ));


    }
}
