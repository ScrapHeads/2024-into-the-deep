package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public final class MecanumLocalizerInputsMessage {
    public long timestamp;
    public PositionVelocityPair leftFront;
    public PositionVelocityPair leftBack;
    public PositionVelocityPair rightBack;
    public PositionVelocityPair rightFront;
    public double yaw;
    public double pitch;
    public double roll;

    public MecanumLocalizerInputsMessage(PositionVelocityPair leftFront, PositionVelocityPair leftBack, PositionVelocityPair rightBack, PositionVelocityPair rightFront, GoBildaPinpointDriver odo) {
        this.timestamp = System.nanoTime();
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;
        {
//            this.yaw = angles.getYaw(AngleUnit.RADIANS);
//            this.pitch = angles.getPitch(AngleUnit.RADIANS);
//            this.roll = angles.getRoll(AngleUnit.RADIANS);

            this.yaw = odo.getHeading();
            this.pitch = 0;
            this.roll = 0;
        }
    }
}
