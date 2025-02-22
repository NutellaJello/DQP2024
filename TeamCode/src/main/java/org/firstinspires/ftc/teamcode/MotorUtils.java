package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.BlueSide5Spec;

public class MotorUtils {
    public static void setPositionCustom(LinearOpMode opMode, DcMotorEx motor, double targetPosition, BlueSide5Spec.SimplePIDF pidController, double tolerance) {
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while (Math.abs(targetPosition - motor.getCurrentPosition()) > tolerance && opMode.opModeIsActive()) {
            double power = 5/*pidController.calculate(motor.getCurrentPosition(), targetPosition)*/;
            motor.setPower(power);
        }
        motor.setPower(0); // Stop motor when target is reached
    }
}