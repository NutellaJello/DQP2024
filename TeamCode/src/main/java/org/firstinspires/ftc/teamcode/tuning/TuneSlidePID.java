package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
public class TuneSlidePID extends LinearOpMode {
    private DcMotorEx motor;
    private FtcDashboard dashboard;

    public static double p = 5.0;
    public static double i = 0.0;
    public static double d = 0.5;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "slides");
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            //PIDFCoefficients coefficients = new PIDFCoefficients(p, i, d, 0.0);
            //motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefficients);
            PIDFCoefficients originalPID = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            double targetPosition = -890; // Example target
            motor.setTargetPosition((int) targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1.0);

//            dashboard.getTelemetry().addData("P", p);
//            dashboard.getTelemetry().addData("I", i);
//            dashboard.getTelemetry().addData("D", d);
            dashboard.getTelemetry().addData("P", originalPID.p);
            dashboard.getTelemetry().addData("I", originalPID.i);
            dashboard.getTelemetry().addData("D", originalPID.d);
            dashboard.getTelemetry().addData("forward", originalPID.f);
            dashboard.getTelemetry().addData("Position", motor.getCurrentPosition());
            dashboard.getTelemetry().addData("Target", targetPosition);
            dashboard.getTelemetry().addData("Error", targetPosition - motor.getCurrentPosition());
            dashboard.getTelemetry().update();
        }
    }
}

