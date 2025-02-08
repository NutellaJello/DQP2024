package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BlueSide5Spec;

@Config
@TeleOp(name = "Get & Adjust PIDF", group = "Test")
public class PIDForMotor extends LinearOpMode {

    private DcMotorEx slides;
    private FtcDashboard dashboard;

    // Dashboard adjustable PIDF values
    public static double P = 12.0;
    public static double I = 5.0;
    public static double D = 0.0;
    public static double F = 0.0;

    public static int targetPosition = -300;  // Adjustable target position

    @Override
    public void runOpMode() {
        // Initialize hardware
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get current PIDF from motor controller
        PIDFCoefficients currentPIDF = slides.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Load current PIDF values into dashboard variables
        P = currentPIDF.p;
        I = currentPIDF.i;
        D = currentPIDF.d;
        F = currentPIDF.f;

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();

        waitForStart();
        sleep(7000);
        BlueSide5Spec.SimplePIDF pidController = new BlueSide5Spec.SimplePIDF(P,I, D, F);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            double currentPosition = slides.getCurrentPosition();
            double error = targetPosition - currentPosition;
            double power = pidController.calculate(currentPosition, targetPosition);

            slides.setPower(power);

            if (Math.abs(targetPosition - currentPosition) < 5) {  // Stop when near target
                slides.setPower(0);
                break;
            }

            // FTC Dashboard telemetry packet
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Position", targetPosition);
            packet.put("Current Position", currentPosition);
            packet.put("Error (Target - Current)", error);
            packet.put("P", P);
            packet.put("I", I);
            packet.put("D", D);
            packet.put("F", F);
            dashboard.sendTelemetryPacket(packet);

            // Driver Station telemetry
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Current PIDF", "P: %.2f I: %.2f D: %.2f F: %.2f", P, I, D, F);
            telemetry.update();
        }
    }
}