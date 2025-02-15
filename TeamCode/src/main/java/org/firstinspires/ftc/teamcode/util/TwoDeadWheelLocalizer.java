package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {
        private final double inPerTick = (3.5 * Math.PI / 2000) * 0.36020518;
        public double parYTicks = 4.5 / inPerTick; // y position of the parallel encoder (in tick units)
        // TODO: change perp x ticks and check direction on line 57
        public double perpXTicks = 5.5 / inPerTick; // x position of the perpendicular encoder (in tick units)
        public double millyInInch = 25.4;
    }

    public static Params PARAMS = new Params();

//    public final Encoder par, perp;

//    public final IMU imu;

    public final GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, GoBildaPinpointDriver odo, double inPerTick) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
//        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));

//        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));

        // TODO: reverse encoder directions if needed
//        par.setDirection(DcMotorSimple.Direction.REVERSE);
//        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.update();

        this.odo = odo;

        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
//        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
//        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        odo.update();

        Pose2D pos = odo.getPosition();

        PositionVelocityPair parPosVel = new PositionVelocityPair(
                (int) (pos.getX(DistanceUnit.INCH)),
                (int) (odo.getVelX() / PARAMS.millyInInch),
                0,
                0);

        PositionVelocityPair perpPosVel = new PositionVelocityPair(
                (int) (pos.getY(DistanceUnit.INCH)),
                (int) (odo.getVelY() / PARAMS.millyInInch),
                0,
                0);

//        parPosVel = new PositionVelocityPair(
//                (int) (parPosVel.position * PARAMS.parXMult),
//                (int) (parPosVel.velocity * PARAMS.parXMult),
//                parPosVel.rawPosition,
//                parPosVel.rawVelocity);
//
//        perpPosVel = new PositionVelocityPair(
//                (int) (perpPosVel.position * PARAMS.perpYMult),
//                (int) (perpPosVel.velocity * PARAMS.perpYMult),
//                perpPosVel.rawPosition,
//                perpPosVel.rawVelocity);

//        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double headingRadian = odo.getHeading();

        // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
//        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        Pose2D forwardBackVelocity = odo.getVelocity(); //Unit per second
        double headingVelocity = odo.getHeadingVelocity(); //Radians per second


//        AngularVelocity angularVelocity = new AngularVelocity(
//                UnnormalizedAngleUnit.RADIANS,
//                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
//                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
//                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
//                angularVelocityDegrees.acquisitionTime
//        );

        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS",
                new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, odo));

//        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        Rotation2d heading = Rotation2d.exp(headingRadian);


        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
//        double rawHeadingVel = angularVelocity.zRotationRate;
        double rawHeadingVel = headingVelocity;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = (int) pos.getX(DistanceUnit.INCH);
            lastPerpPos = (int) pos.getY(DistanceUnit.INCH);
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parPosDelta = (int) pos.getX(DistanceUnit.INCH) - lastParPos;
        int perpPosDelta = (int) pos.getY(DistanceUnit.INCH) - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

//        Twist2dDual<Time> twist = new Twist2dDual<>(
//                new Vector2dDual<>(
//                        new DualNum<Time>(new double[] {
//                                parPosDelta - PARAMS.parYTicks * headingDelta,
//                                odo.getVelX() - PARAMS.parYTicks * headingVel,
//                        }).times(inPerTick),
//                        new DualNum<Time>(new double[] {
//                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
//                                odo.getVelY() - PARAMS.perpXTicks * headingVel,
//                        }).times(inPerTick)
//                ),
//                new DualNum<>(new double[] {
//                        headingDelta,
//                        headingVel,
//                })
//        );

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta * headingDelta,
                                (odo.getVelX() / PARAMS.millyInInch) * headingVel,
                        }),
                        new DualNum<Time>(new double[] {
                                perpPosDelta * headingDelta,
                                (odo.getVelY() / PARAMS.millyInInch) * headingVel,
                        })
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = (int) pos.getX(DistanceUnit.INCH);
        lastPerpPos = (int) pos.getY(DistanceUnit.INCH);
        lastHeading = heading;

        return twist;
    }
}
