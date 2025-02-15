package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

public class FollowDrivePathPP extends CommandBase {
    private Action path = null;
    private PinpointDrive pinpointDrive = null;

    private boolean isFinished = false;

    public FollowDrivePathPP(PinpointDrive pinpointDrive, Action path) {
        this.pinpointDrive = pinpointDrive;
        this.path = path;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        isFinished = path.run(packet);
        packet.put("test", true);
        packet.put("path", path);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean isInterrupted) {
        pinpointDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    @Override
    public boolean isFinished() {
        return !isFinished;
    }
}
