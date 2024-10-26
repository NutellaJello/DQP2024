package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


// this teleop is fieldcentric, when joystick moves up, the robot will always more forward
@TeleOp (name="DQP 2024 teleop")
public class teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        double dampS = 0.85;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL"); //0
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR"); //1
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BL"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BR"); //3

        //initalizing for the cf intake, may have to reverse motor

        DcMotor motorCF = hardwareMap.dcMotor.get("CF"); // expansion port 0
//
//        Servo cfLateral = hardwareMap.servo.get("");
//        Servo cfLI = hardwareMap.servo.get("");
//        Servo cfRI = hardwareMap.servo.get("");
        // CRServo makes the servo go past 360, makes it continuous


       // CRServo slide = (CRServo) hardwareMap.servo.get("slide"); //0
        Servo cfLateral = hardwareMap.servo.get("cfLateral"); //1
        CRServo rclaw = hardwareMap.get(CRServo.class, "rightclaw"); // CH port 2
        CRServo lclaw = hardwareMap.get(CRServo.class, "leftclaw"); // CH port 3

// Reverse the direction of the left claw
        lclaw.setDirection(CRServo.Direction.REVERSE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double cfPosition = 0.13;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//            telemetry.addData("claw Postition: ", Double.toString(claw.getPosition()));
//            telemetry.addData("rotational Postition: ", Double.toString(rotclaw.getPosition()));
            telemetry.addData("CFlateral Postition: ", Double.toString(cfLateral.getPosition()));




            // origonally y is neg, x is pos
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            // We have to reset the yaw often since of the joystick drift
            if (gamepad1.left_bumper) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower* dampS);
            motorBackLeft.setPower(backLeftPower* dampS);
            motorFrontRight.setPower(frontRightPower* dampS);
            motorBackRight.setPower(backRightPower* dampS);

            // if gamepad2 y stick moves, cf moves
            motorCF.setPower(gamepad2.left_stick_y * dampS);


            if(gamepad2.left_trigger> 0){
                lclaw.setPower(0.6);
                rclaw.setPower(0.6);
            }
            else if (gamepad2.right_trigger > 0){
                lclaw.setPower(-0.1);
                rclaw.setPower(-0.1);
            }
            else{
                lclaw.setPower(0);
                rclaw.setPower(0);
            }


            if(gamepad2.dpad_up){
                cfPosition = 1;
            }
            else if (gamepad2.dpad_down) {
                cfPosition = 0.07;
            }
            else if (gamepad2.dpad_right){
                cfPosition = 0.13;
            }
            // 0.13 bar height
            cfLateral.setPosition(cfPosition);


            /*
            // for the opening claw, use left bumper on gamepad2
            double clawPosition = 0.4;

            if (gamepad2.left_bumper){
                clawPosition += 0.05;
            }
            claw.setPosition(clawPosition);




            // for rotational, dpadleft rotates it counterclock, dp right rotate clock
            double rpos = 0.4;
            if (gamepad2.dpad_left){
                 rpos -= 0.05;
            }
            else if(gamepad2.dpad_right){
                rpos += 0.05;
            }
            rotclaw.setPosition(rpos);


            // up down claw
            double lpos = 0.4;
            if (gamepad2.y){
                lpos -= 0.05;
            }
            else if(gamepad2.a){
                lpos += 0.05;
            }
            latclaw.setPosition(lpos);



             */
            telemetry.update();
        }
    }
}