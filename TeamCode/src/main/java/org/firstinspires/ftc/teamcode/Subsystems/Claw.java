package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Claw implements Subsystem {
    //Setting clawIntake variable to be set in the Claw function
    private final ServoEx clawIntake;

    private final DigitalChannel touchSensor;

    private boolean isPressed;

    public Claw() {
        //Linking clawIntake in the code to the servo on the robot
        clawIntake = new SimpleServo(hm, "intakeClaw", -1, 1);

        touchSensor = hm.get(DigitalChannel.class, "touchClaw");

    }

    @Override
    public void periodic() {
        // add telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Touch Sensor",touchSensor.getState());
        packet.put("Claw Pos", clawIntake.getPosition());
        dashboard.sendTelemetryPacket(packet);
    }

    public boolean getTouchSensor() {
        return !touchSensor.getState();
    }

    public void setPower(double pos) {
        //Setting the clawIntake to constantly move
        clawIntake.turnToAngle(pos);
    }
}
