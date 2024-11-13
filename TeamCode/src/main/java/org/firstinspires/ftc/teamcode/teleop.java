package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp (name="Field Centric Teleop")
public class teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double dampS = 0.85;
        double dampSpeedRatio = 0.8;
        double dampTurnRatio  = 1;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL"); //0
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR"); //1
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BL"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BR"); //3

        //initalizing for the cf intake
        DcMotor motorCF = hardwareMap.dcMotor.get("CF"); // expansion port 0



        // all servos for claw
        //expansion
        Servo cfLateral = hardwareMap.servo.get("cfLateral"); //0
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

        pivot.setDirection(Servo.Direction.REVERSE);

        double cfPosition = 0.07;

        // temp testing
        claw.setPosition(0.3);
        rotation.setPosition(0);
        pivot.setPosition(0.3);
        double pivotpos = 0.55;

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
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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


            motorCF.setPower(gamepad2.left_stick_y * dampS);


            if (gamepad2.right_stick_button){
                pivotpos=0.525;
            }
            pivotpos += ( 0.005* gamepad2.right_stick_x);
            pivot.setPosition(pivotpos);


            if(gamepad2.dpad_up){
                rotation.setPosition(0.73);
            }
            if(gamepad2.dpad_down){
                rotation.setPosition(0.05);
            }
            if(gamepad2.dpad_right){
                rotation.setPosition(0.39);
            }
            //open claw is left bumper, right is close
            if (gamepad2.left_bumper){
                claw.setPosition(0.58);
            }
            if(gamepad2.right_bumper){
                claw.setPosition(0.08);
            }


            if (gamepad2.y) {
                cfPosition = 0.95;
            } else if (gamepad2.a) {
                cfPosition = 0.07;
            }
            else if (gamepad2.b){
                cfPosition = 0.3;
            }
            // 0.13 bar height
            cfLateral.setPosition(cfPosition);

            telemetry.update();
        }
    }
}