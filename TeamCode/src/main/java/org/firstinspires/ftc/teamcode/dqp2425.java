package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "dqp2425", group = "TeleOp")

public class dqp2425 extends LinearOpMode{
    private DcMotor slides;
    private Servo slides2;
    private Servo rotation;
    private Servo pivot;
    private Servo claw;
    private Servo claw2;
    private Servo rotation2;
    private DcMotor hang;

    @Override
    public void runOpMode() {
        double dampS = 0.85;
        double dampSpeedRatio = 0.8;
        double dampTurnRatio  = 1;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL"); //0
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR"); //1
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BL"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BR"); //3
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        double tgtPower = 0;
        slides=hardwareMap.get(DcMotor.class, "slides");// outtake, EPM 0
        slides2=hardwareMap.get(Servo.class, "slides2");// intake, EPS 0 "AXONMAX"
        rotation=hardwareMap.get(Servo.class, "rotation"); // EPS 3
        pivot=hardwareMap.get(Servo.class, "pivot"); // EPS 2
        claw=hardwareMap.get(Servo.class, "claw"); // EPS 1
        claw2=hardwareMap.get(Servo.class, "claw2"); // EPS 4
        rotation2=hardwareMap.get(Servo.class, "rotation2"); // EPS 5
        hang=hardwareMap.get(DcMotor.class, "hang"); // EPM 1

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        double pivotpos=0.5;
        double clawpos=0.2;
        double rotpos=1;
        double claw2pos=0;
        double rot2pos=0;
        double slides2pos=0.69;
        while (opModeIsActive()) {
            tgtPower=this.gamepad2.left_stick_y;
            telemetry.addData("slides2", slides2pos);
            telemetry.addData("claw", clawpos);
            telemetry.addData("pivot", pivotpos);
            telemetry.addData("rotation", rotpos);
            telemetry.addData("claw2", claw2pos);
            telemetry.addData("rotation2", rot2pos);
            telemetry.addData("outtake",slides.getCurrentPosition());
            telemetry.update();
            if (gamepad1.left_bumper) {
                imu.resetYaw();
            }

            double y = Range.clip(-gamepad1.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(-gamepad1.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(-gamepad1.right_stick_x, -1, 1);

            //    double arct = 0;

            if(gamepad1.right_bumper){
                dampSpeedRatio = 0.2;
                dampTurnRatio = 0.3;
            }else{
                dampSpeedRatio = 0.8;
                dampTurnRatio = 0.8;
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
            motorFrontLeft.setPower(flPower);
            motorBackLeft.setPower(blPower);
            motorFrontRight.setPower(frPower);
            motorBackRight.setPower(brPower);
            double idkman=-this.gamepad2.left_stick_y;
            if (idkman>0) {
                if (slides2pos>0.43) slides2pos-=idkman/1000;
                else slides2pos=0.43;
            }
            if (idkman<0) {
                if (slides2pos<0.69) slides2pos-=idkman/1000;
                else slides2pos=0.69;
            }
            if(this.gamepad2.left_stick_button){
                slides2pos=0.69;
            }
            if(gamepad2.dpad_up){
                transfer();
            }
            if (gamepad2.dpad_down){
                long starttime = System.currentTimeMillis();
                // makes sure outtake is in position
                rotation2.setPosition(0);
                claw2.setPosition(0.54);
                // moves slides down to location
                slides.setTargetPosition(30);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(0.5);
                // make sure pivot is aligned
                pivot.setPosition(0.5);
                rotation.setPosition(0.28);
                slides2pos=0.68;

             //   transfer();

            }
            slides2.setPosition(slides2pos);

           // slides.setPower(this.gamepad2.right_stick_y);

            if (gamepad2.right_stick_y > 0) {
                slides.setTargetPosition(slides.getCurrentPosition() +100);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(gamepad2.right_stick_y * 1.2);
            }
            if (gamepad2.right_stick_y < 0) {
                slides.setTargetPosition(slides.getCurrentPosition() -100);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(gamepad2.right_stick_y);
            }



            if (this.gamepad1.right_trigger>0) {
                hang.setPower(this.gamepad1.right_trigger/2.0);
            }
            if (this.gamepad1.left_trigger>0) {
                hang.setPower(-this.gamepad1.left_trigger/.0);
            }
//            pivot.setPosition(0);
            if (gamepad2.dpad_right) {
//                if (rotpos<1) rotpos+=0.001;
//                else rotpos=1;
                rotpos=0.95;
                rotation.setPosition(rotpos);
            }
            if (gamepad2.dpad_left) {
//                if (rotpos>0) rotpos-=0.001;
//                else rotpos=0;
                rotpos=0.28;
                rotation.setPosition(rotpos);
            }
            if (gamepad2.left_trigger>0) {
//                if (claw2pos<1) claw2pos+=0.001;
//                else claw2pos=1;
                claw2pos=0.54;
                claw2.setPosition(claw2pos);
            }
            if (gamepad2.right_trigger>0) {
//                if (claw2pos>0) claw2pos-=0.001;
//                else claw2pos=0;
                claw2pos=0.25;
                claw2.setPosition(claw2pos);
            }
            if (gamepad2.x) {
//                if (clawpos>0) clawpos-=0.001;
//                else clawpos=0;
                clawpos=0.08;
                claw.setPosition(clawpos);
            }
            if (gamepad2.y) {
//                if (clawpos<0.5) clawpos+=0.001;
//                else clawpos=0.5;
                clawpos=0.41;
                claw.setPosition(clawpos);
            }
            if (gamepad2.a) {
                if (pivotpos<0.9) pivotpos+=0.001;
                else pivotpos=0.9;
                pivot.setPosition(pivotpos);
            }
            if (gamepad2.b) {
                if (pivotpos>0.1) pivotpos-=0.001;
                else pivotpos=0.1;
                pivot.setPosition(pivotpos);
            }
            if (gamepad2.left_bumper) {
//                if (rot2pos>0) rot2pos-=0.001;
//                else rot2pos=0;
                rot2pos=0;
                rotation2.setPosition(rot2pos);
            }
            if (gamepad2.right_bumper) {
//                if (rot2pos<1) rot2pos+=0.001;
//                else rot2pos=1;
                rot2pos=0.85;
                rotation2.setPosition(rot2pos);
            }

//            if (gamepad2.x) claw.setPosition(0);
//            if (gamepad2.y) claw.setPosition(0.2);


        }

    }
    public void transfer(){

        claw2.setPosition(0.25);
        sleep(5200);
        claw.setPosition(0.41);

    }
}
