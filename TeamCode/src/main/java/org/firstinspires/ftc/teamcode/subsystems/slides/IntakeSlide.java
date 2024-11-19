package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSlide {
    private Servo intakeSlide;
    private double intakepos;
    public IntakeSlide(HardwareMap hardwareMap){
        intakeSlide =hardwareMap.get(Servo.class, "slides2");// intake, EPS 0 "AXONMAX"
    }
    public void Teleop(Gamepad gamepad, Telemetry telemetry) {
        Teleop(gamepad, telemetry, true);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean showTelemetry){ //Code to be run in Teleop Mode void Loop at top level
        double joystick = -gamepad.left_stick_y;
        if (joystick>0) {
            if (intakepos>0.43) intakepos-=joystick/1000;
            else intakepos=0.43;
        }
        if (joystick<0) {
            if (intakepos<0.71) intakepos-=joystick/1000;
            else intakepos=0.71;
        }
        if(gamepad.left_stick_button){
            intakepos=0.70;
        }

        intakeSlide.setPosition(intakepos);


        if(showTelemetry) {
            telemetry.addData("FL Power", intakeSlide.getPosition());
        }

    }
    public void setPosition(double value) {
        intakepos = value;
        intakeSlide.setPosition(value);
    }
}
