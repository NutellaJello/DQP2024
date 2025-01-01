package org.firstinspires.ftc.teamcode.subsystems.Rotations;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakePitchArm {
    private double rotpos= 1;
    private double rotin = 0.28;
    private double rotout = 0.95;

    private Servo intakePitchArm;
    public IntakePitchArm(HardwareMap hardwareMap){
        intakePitchArm =hardwareMap.get(Servo.class, "rotation");// intake pitch arm, EPS 3
    }

    public void Initialize()
    {
        //intakePitchArm.setPosition(0.28);
        intakePitchArm.setPosition(rotin);
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry) {
        Teleop(gamepad2, telemetry, true);
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry, boolean showTelemetry) {
        if (gamepad2.a) {
            rotpos=rotout;
            intakePitchArm.setPosition(rotpos);
            }
        if (gamepad2.dpad_left) {
            rotpos=rotin;
            intakePitchArm.setPosition(rotpos);
            }
        if(showTelemetry)
            telemetry.addData("rotation", rotpos);
    }

    public void SetPitchPosition(double position){
        rotpos = position;
        intakePitchArm.setPosition(rotpos);
    }

    public double GetPitchPosition()
    {
        return rotpos;
    }

    public void PitchDown(){
        rotpos=rotout-0.1;
        intakePitchArm.setPosition(rotpos);
    }
    public void PitchIn(){
        rotpos=rotin;
        intakePitchArm.setPosition(rotpos);
    }
    public void PitchOut(){
        rotpos=rotout;
        intakePitchArm.setPosition(rotpos);
    }
    public boolean PositionReachedWithTolerance(double tolerance){
        return (intakePitchArm.getPosition()-rotpos)<= tolerance;
    }
}
