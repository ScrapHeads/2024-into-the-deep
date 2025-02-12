package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public final class TwoDeadWheelInputsMessage {
    public long timestamp;
    public PositionVelocityPair par;
    public PositionVelocityPair perp;
    public double yaw;
    public double pitch;
    public double roll;
    public double xRotationRate;
    public double yRotationRate;
    public double zRotationRate;

    public TwoDeadWheelInputsMessage(PositionVelocityPair par, PositionVelocityPair perp, GoBildaPinpointDriver odo) {
        this.timestamp = System.nanoTime();
        this.par = par;
        this.perp = perp;
        {
            this.yaw = odo.getHeading();
            this.pitch = 0;
            this.roll = 0;
        }
        {
//            this.xRotationRate = angularVelocity.xRotationRate;
//            this.yRotationRate = angularVelocity.yRotationRate;
//            this.zRotationRate = angularVelocity.zRotationRate;

            this.xRotationRate = odo.getHeadingVelocity();
            this.yRotationRate = 0;
            this.zRotationRate = 0;
        }
    }
}
