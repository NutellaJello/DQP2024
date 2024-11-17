package org.firstinspires.ftc.teamcode.subsystems.Claws;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawBase {
    private Servo claw;
    private double claw2pos;
    public ClawBase(HardwareMap hardwareMap, String clawName){
        claw = hardwareMap.get(Servo.class, clawName);
    }

    public void openBy(double value) {
        claw2pos = value;
        claw.setPosition(value);
    }

    public void closeBy(double value) {
        claw2pos = value;
        claw.setPosition(value);
    }

    public void setPosition(double value) {
        claw2pos = value;
        claw.setPosition(value);
    }

    public boolean clawReachesPosWithinBandOf(double band){
        return Math.abs(claw.getPosition()-claw2pos)<band;
    }

    public double getPosition() {
        return claw.getPosition();
    }
}
