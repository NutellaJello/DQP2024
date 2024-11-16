package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {
    // Instantiate the drivetrain motor variables
    private DcMotorEx fl; //Front left motor of drivetrain
    private DcMotorEx fr; //Front right motor of drivetrain
    private DcMotorEx bl; //Back left motor of drivetrain
    private DcMotorEx br; //Back right motor of drivetrain
    private double dampSpeedRatio = 0.8;
    private double dampTurnRatio  = -1;

    public Drivetrain(HardwareMap hardwareMap){                 // Motor Mapping
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        br = hardwareMap.get(DcMotorEx.class, "BR");

        // Set motor direction based on which side of the robot the motors are on
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.FORWARD);
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry) {
        Teleop(gamepad, telemetry, false);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean showTelemetry){ //Code to be run in Teleop Mode void Loop at top level
        double y = Range.clip(-gamepad.left_stick_y, -1, 1);
        //left stick x value
        double x = Range.clip(-gamepad.left_stick_x, -1, 1);
        //right stick x value
        double rx = Range.clip(-gamepad.right_stick_x, -1, 1);

        //    double arct = 0;

        if(gamepad.right_bumper){
            dampSpeedRatio = 0.2;
            dampTurnRatio = -0.3;
        }else{
            dampSpeedRatio = 0.8;
            dampTurnRatio = -0.8;
        }

        double flPower = (y - x) * dampSpeedRatio + dampTurnRatio * rx;
        double frPower = (y + x) * dampSpeedRatio - dampTurnRatio * rx;
        double blPower = (y + x) * dampSpeedRatio + dampTurnRatio * rx;
        double brPower = (y - x) * dampSpeedRatio - dampTurnRatio * rx;

        double maxFront = Math.max(flPower, frPower);
        double maxBack = Math.max(blPower, brPower);
        double maxPower = Math.max(maxFront, maxBack);

        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }
        //finally moving the motors
        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);

        if(showTelemetry) {
            telemetry.addData("FL Power", flPower);
            telemetry.addData("BL Power", blPower);
            telemetry.addData("FR Power", frPower);
            telemetry.addData("BR Power", brPower);
        }

    }

}
