package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeSlide {
    private DcMotor outtakeSlide;
    private double outtakepos;
    public OuttakeSlide(HardwareMap hardwareMap){
        outtakeSlide =hardwareMap.get(DcMotor.class, "slides");// outtake, EPS 0 "AXONMAX"
    }

    public void Initalize()
    {
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Teleop(Gamepad gamepad, Telemetry telemetry) {
        Teleop(gamepad, telemetry, true);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean showTelemetry){ //Code to be run in Teleop Mode void Loop at top level
        double joystick = -gamepad.left_stick_y;
        if (joystick>0) {
            if (outtakepos>0.43) outtakepos-=joystick/1000;
            else outtakepos=0.43;
        }
        if (joystick<0) {
            if (outtakepos<0.71) outtakepos-=joystick/1000;
            else outtakepos=0.71;
        }
        if(gamepad.left_stick_button){
            outtakepos=0.70;
        }

        //outtakeSlide.setPosition(outtakepos);


        if(showTelemetry) {
            //telemetry.addData("FL Power", outtakeSlide.getPosition());
        }

    }
    public void setPosition(double value) {
        outtakepos = value;
        //outtakeSlide.setPosition(value);
    }
}
