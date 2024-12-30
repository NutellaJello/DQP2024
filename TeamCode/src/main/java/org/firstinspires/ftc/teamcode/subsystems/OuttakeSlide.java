package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeSlide {
    // Instantiate the drivetrain motor variables
    private DcMotorEx outtakeSlide; //Front left motor of drivetrain


    public OuttakeSlide(HardwareMap hardwareMap){                 // Motor Mapping
       outtakeSlide = hardwareMap.get(DcMotorEx.class, "slides");

       outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry) {
        Teleop(gamepad, telemetry, true);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean showTelemetry){ //Code to be run in Teleop Mode void Loop at top level
        outtakeSlide.setPower(1);
        if (gamepad.right_stick_y > 0) {
            outtakeSlide.setTargetPosition(outtakeSlide.getCurrentPosition() +100);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad.right_stick_y < 0) {
            outtakeSlide.setTargetPosition(outtakeSlide.getCurrentPosition() -100);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //slides.setPower(gamepad2.right_stick_y);
        }
        if(gamepad.y){
            outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(showTelemetry) {
            telemetry.addData("Outtake Slides Position: ", outtakeSlide.getCurrentPosition());
        }

    }

}