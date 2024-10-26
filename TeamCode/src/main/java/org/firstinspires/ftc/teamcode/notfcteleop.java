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


// this teleop is  not fieldcentric
@TeleOp
public class notfcteleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        double dampS = 0.85;
        double dampSpeedRatio = 0.6;
        double dampTurnRatio  = 1;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL"); //0
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR"); //1
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BL"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BR"); //3

        //initalizing for the cf intake
     //   DcMotor motorCF = hardwareMap.dcMotor.get("CF"); // expansion port 0
        // CRServo makes the servo go past 360, makes it continuous


        // all servos for claw
        //expansion
 //       Servo cfLateral = hardwareMap.servo.get("cfLateral"); //0
        Servo claw = hardwareMap.servo.get("claw"); // 1
        Servo pivot = hardwareMap.servo.get ("pivot"); //  2
        Servo rotation = hardwareMap.servo.get("rotation"); // 3

// Reverse the direction of the left claw
      //  lclaw.setDirection(CRServo.Direction.REVERSE);)

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double cfPosition = 0.07;

        // temp testing
        claw.setPosition(0.3);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("claw Postition: ", Double.toString(claw.getPosition()));
            telemetry.addData("rotational Postition: ", Double.toString(rotation.getPosition()));
      //      telemetry.addData("CFlateral Postition: ", Double.toString(cfLateral.getPosition()));


            // origonally y is neg, x is pos
//            double y = -gamepad1.left_stick_y;
//            double x = -gamepad1.left_stick_x;
//            double rx = -gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            // We have to reset the yaw often since of the joystick drift
            if (gamepad1.left_bumper) {
                imu.resetYaw();
            }

            double y = Range.clip(-gamepad1.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(gamepad1.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(-gamepad1.right_stick_x, -1, 1);

            //    double arct = 0;

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
            motorFrontLeft.setPower(flPower);
            motorBackLeft.setPower(blPower);
            motorFrontRight.setPower(frPower);
            motorBackRight.setPower(brPower);




      //      motorCF.setPower(gamepad2.left_stick_y * dampS);

            double rotationpos = 0.3;
            double pivotpos = 0.4;

            if (gamepad2.dpad_left){
                pivotpos += 0.05;
            }
            else if (gamepad2.dpad_right){
                pivotpos -= 0.05;
            }
            pivot.setPosition(pivotpos);

            if(gamepad2.y){
                rotationpos += 0.05;
                //rotation.setPosition();
            }
            else if(gamepad2.a){
                rotationpos -=0.05;
                //rotation.setPosition();
            }
            rotation.setPosition(rotationpos);
            //open claw is left bumper, right is close
            if (gamepad2.left_bumper){
                claw.setPosition(0.5);
            }
            if(gamepad2.right_bumper){
                claw.setPosition(0.2);
            }


            if (gamepad2.y) {
                cfPosition = 1;
            } else if (gamepad2.a) {
                cfPosition = 0.07;
            }
            else if (gamepad2.b){
                cfPosition = 0.13;
            }
            // 0.13 bar height
     //       cfLateral.setPosition(cfPosition);

            telemetry.update();
        }
    }
}