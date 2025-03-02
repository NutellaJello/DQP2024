package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp(name = "Motor Speed Test", group = "Testing")
public class SimpleTestMotor extends LinearOpMode {

    private DcMotorEx leftFront, rightFront, leftBack;
    private static final double TEST_POWER = 0.1; // Set to 50% power for test
    private static final double TICKS_PER_REV = 537.6; // Adjust based on motor type
    private static final double SECONDS_PER_MINUTE = 60.0;

    @Override
    public void runOpMode() {
        // Initialize motors as DcMotorEx to access getVelocity()
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");

        // Ensure motors use encoders
        /*for (DcMotorEx motor : new DcMotorEx[]{leftFront, rightFront, leftBack}) {
           // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/

        waitForStart();

        // Apply test power
        for (DcMotorEx motor : new DcMotorEx[]{leftFront, rightFront, leftBack}) {
            motor.setPower(TEST_POWER);
        }

        while (opModeIsActive()) {
            // Read velocity and convert ticks/sec to RPM
            double leftFrontRPM = leftFront.getVelocity();
            double rightFrontRPM = rightFront.getVelocity();
            double leftBackRPM = rightFront.getVelocity();

            // Display results
            telemetry.addData("Power", "Testing at %.2f", TEST_POWER);
            telemetry.addData("LF Motor (RPM)", "%.2f", leftFrontRPM);
            telemetry.addData("RF Motor (RPM)", "%.2f", rightFrontRPM);
            telemetry.addData("LB Motor (RPM)", "%.2f", leftBackRPM);

            telemetry.update();
//            leftFront.setPower(0.1);
//            rightFront.setPower(0.1);
//            leftBack.setPower(0.1);
        }

        // Stop all motors after test
        for (DcMotorEx motor : new DcMotorEx[]{leftFront, rightFront, leftBack}) {
            motor.setPower(0);
        }
    }
}

