package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain {
    // Instantiate the drivetrain motor variables
    private DcMotorEx fl; //Front left motor of drivetrain
    private DcMotorEx fr; //Front right motor of drivetrain
    private DcMotorEx bl; //Back left motor of drivetrain
    private DcMotorEx br; //Back right motor of drivetrain
    private IMU imu;
    private double dampSpeedRatio;
    private double dampTurnRatio;

    public Drivetrain(HardwareMap hardwareMap){
        // Motor Mapping
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        br = hardwareMap.get(DcMotorEx.class, "BR");

        // Set motor direction based on which side of the robot the motors are on
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.FORWARD);
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean fieldCentric) {
        Teleop(gamepad, telemetry, false, fieldCentric);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean showTelemetry, boolean fieldCentric){ //Code to be run in Teleop Mode void Loop at top level
        // field centric driving
        if (fieldCentric){
            if (gamepad.dpad_up){
                imu.resetYaw();
            }

            // get heading
            YawPitchRollAngles robotOrientation;
            robotOrientation = imu.getRobotYawPitchRollAngles();
            double heading   = robotOrientation.getYaw(AngleUnit.RADIANS);

            double y = Range.clip(-gamepad.left_stick_y * Math.cos(heading) + -gamepad.left_stick_x * Math.sin(heading), -1, 1);
            //left stick x value

            double x = Range.clip( -gamepad.left_stick_y * Math.sin(heading) + gamepad.left_stick_x * Math.cos(heading), -1, 1);
            //right stick x value

            double rx = Range.clip(gamepad.right_stick_x, -1, 1);


            if(gamepad.right_bumper){
                dampSpeedRatio = 0.33;
                dampTurnRatio = 0.22;
            }else{
                dampSpeedRatio = 1;
                dampTurnRatio = 0.75;
            }

            double flPower = (y + x) * dampSpeedRatio + dampTurnRatio * rx;
            double frPower = (y - x) * dampSpeedRatio - dampTurnRatio * rx;
            double blPower = (y - x) * dampSpeedRatio + dampTurnRatio * rx;
            double brPower = (y + x) * dampSpeedRatio - dampTurnRatio * rx;

            double maxPower;
            maxPower = Math.max(Math.abs(flPower), Math.abs(frPower));
            maxPower = Math.max(maxPower, Math.abs(blPower));
            maxPower = Math.max(maxPower, Math.abs(brPower));

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
        // normal driving
        else {
            double y = Range.clip(-gamepad.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(-gamepad.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(gamepad.right_stick_x, -1, 1);

            if (gamepad.right_bumper) {
                dampSpeedRatio = 0.33;
                dampTurnRatio = 0.22;
            } else {
                dampSpeedRatio = 1;
                dampTurnRatio = 0.75;
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

}
