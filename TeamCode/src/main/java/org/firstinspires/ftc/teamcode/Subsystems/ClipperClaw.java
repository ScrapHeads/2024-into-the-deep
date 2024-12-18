package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClipperClaw implements Subsystem {
    //Setting clawIntake variable to be set in the Claw function
    private final ServoEx clawClipper;

    public ClipperClaw() {
        //Linking clawIntake in the code to the servo on the robot
        clawClipper = new SimpleServo(hm, "clipperClaw", -80, 80, AngleUnit.DEGREES);
//        clawClipper.turnToAngle(0);
//        clawClipper.setPosition(0);
    }

    @Override
    public void periodic() {
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Claw Pos", clawClipper.getPosition());
        dashboard.sendTelemetryPacket(packet);
    }

    public void setPower(double pos) {
        //Setting the clawIntake to constantly move
//        clawClipper.turnToAngle(pos);
        clawClipper.setPosition(pos);
    }
}