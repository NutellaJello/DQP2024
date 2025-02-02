package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Config
@TeleOp(name = "tunePID", group = "TeleOp")
public class PIDForMotor extends LinearOpMode {
    private DcMotorEx motor;
    private FtcDashboard dashboard;

    public static double p = 10.0;
    public static double i = 0.05;
    public static double d = 0.0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "slides");
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            // ✅ Dynamically apply updated PID values from FTC Dashboard
            PIDFCoefficients coefficients = new PIDFCoefficients(p, i, d, 0.0);
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, coefficients);

            double targetPosition = -890; // Target slide position
            motor.setTargetPosition((int) targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1.0);

            double currentPosition = motor.getCurrentPosition();
            double error = targetPosition - currentPosition;

            // 📊 Send data to FTC Dashboard for graphing
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("P", p);
            packet.put("I", i);
            packet.put("D", d);
            packet.put("Position", currentPosition);
            packet.put("Target", targetPosition);
            packet.put("Error", error);
            dashboard.sendTelemetryPacket(packet); // Send data for real-time graphing

            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.update();
        }
    }
}