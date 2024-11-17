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
    double pivotpos=0.437;
    double clawpos=0.2;
    double rotpos=1;
    double claw2pos=0;
    double rot2pos=0;
    double slides2pos=0.68;

    double f = 0;
    int c1=0;
    int c2=0;
    int c3=0;

    int a1=0;
    double a2=0;


    @Override
    public void runOpMode() {
        double dampS = 0.85;
        double dampSpeedRatio = 0.8;
        double dampTurnRatio  = -0.6;
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
        rotation.setPosition(0.28);
        rotation2.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            tgtPower=this.gamepad2.left_stick_y;
            telemetry.addData("slides2", slides2pos);
            telemetry.addData("claw", clawpos);
            telemetry.addData("pivot", pivotpos);
            telemetry.addData("rotation", rotpos);
            telemetry.addData("claw2", claw2pos);
            telemetry.addData("rotation2", rot2pos);
            telemetry.addData("outtake",slides.getCurrentPosition());
            telemetry.addData("a1", a1);
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
                dampTurnRatio = -0.16;
            }else{
                dampSpeedRatio = 0.8;
                dampTurnRatio = -0.6;
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
                if (slides2pos<0.71) slides2pos-=idkman/1000;
                else slides2pos=0.71;
            }


            slides2.setPosition(slides2pos);



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
            telemetry.addData("slides", slides.getCurrentPosition());



            if (this.gamepad1.right_trigger>0) {
                hang.setPower(this.gamepad1.right_trigger/2.0);
            }
            if (this.gamepad1.left_trigger>0) {
                hang.setPower(-this.gamepad1.left_trigger/3.0);
            }
//
            if (gamepad2.a) {
                rotpos=0.97;
                rotation.setPosition(rotpos);
            }
            if (gamepad2.dpad_left) {
                rotpos=0.28;
                rotation.setPosition(rotpos);
            }
            if (gamepad2.right_bumper) {

                claw2pos=0.54;
                claw2.setPosition(claw2pos);
            }
            if (gamepad2.left_bumper) {

                claw2pos=0.25;
                claw2.setPosition(claw2pos);
            }
            if (gamepad2.x) {
                clawpos=0.15;
                claw.setPosition(clawpos);
            }
            if (gamepad2.y) {
                clawpos=0.47;
                claw.setPosition(clawpos);
            }
            pivotpos+=gamepad2.left_stick_x*0.01;
            pivot.setPosition(pivotpos);
            if (pivotpos>0.9) {
                pivotpos=0.9;
            }
            if (pivotpos<0.2){
                pivotpos=0.2;
            }
            if (gamepad2.left_trigger>0) {
                if (rot2pos>0) rot2pos-=gamepad2.left_trigger/100;
                else rot2pos=0;
//                rot2pos=0;
                rotation2.setPosition(rot2pos);
            }
            if (gamepad2.right_trigger>0) {
                if (rot2pos<0.985) rot2pos+=gamepad2.right_trigger/100;
                else rot2pos=0.985;
//                rot2pos=0.85;
                rotation2.setPosition(rot2pos);
            }

            if(gamepad2.left_stick_button){
                slides.setTargetPosition(-1338);
                rot2pos=0.0385;
                rotation2.setPosition(rot2pos);
            }
//
            // AUTOMATION STUFF
            if (gamepad2.right_stick_button) {
                //transfer();
                slides.setTargetPosition(0);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(0.6);
                claw2pos=0.54;
                rot2pos=0.985;
                rotation2.setPosition(rot2pos);
                claw2.setPosition(claw2pos);
                clawpos=0.08;
                claw.setPosition(clawpos);
                a1=1;
            }
            if(a1==1){
                a2+=0.015;
                if(a2>=0.3) {

                    if( (claw.getPosition()-clawpos)<= 0.0001 ){
                        a1=2;
                        rotpos=0.28;
                        rotation.setPosition(rotpos);
                        pivotpos=0.437;
                        pivot.setPosition(pivotpos);
                    }

                }

            }

            if(a1==2){
                if( (rotation.getPosition()-rotpos)<= 0.00001 ){
                    a1=3;
                    slides2pos=0.70;
                    slides2.setPosition(slides2pos);
                }
            }

            if(a1==3){
                if ((slides2.getPosition() - slides2pos) <= 0.00001) {
                    a2 += 0.015;
                    if (a2 >= 3) {

                        claw2.setPosition(0.25);
                        a1 = 4;
                        a2 = 0;

                    }
                }

            }

            if(a1==4){
                a2+=0.015;
                if(a2>=1) {

                    claw.setPosition(0.47);
                    a1 = 5;
                    a2=0;

                }
            }

            if(a1==5){
                a2+=0.015;
                if(a2>=1) {

                    slides2pos=0.68;
                    slides2.setPosition(slides2pos);
                    a1 = 0;
                    a2=0;

                }
            }



            if (gamepad2.dpad_up) {
                slides.setTargetPosition(-2300);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(0.8);
                c1=1;

            }
            if (c1==1 && Math.abs(slides.getCurrentPosition()+2300)<=100) {
                c1=0;
                rot2pos=0.2513;
                rotation2.setPosition(rot2pos);
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (gamepad2.dpad_down) {
                double rn=this.getRuntime();
                claw2pos=0.54;
                claw2.setPosition(claw2pos);
                //if (this.getRuntime()>rn+1.0) c2=1;
                c2=1;

            }
            if(c2==1){
                f+=0.025;
                if(f>=1){
                    c2=2;
                }
            }
            if (c2==2 && Math.abs(claw2.getPosition()-claw2pos)<=0.000001) {
                c2=0;
                f=0;
                rot2pos=0.989;
                rotation2.setPosition(rot2pos);
                c3=1;

            }
            if (c3==1 && Math.abs(claw2.getPosition()-claw2pos)<0.05 && Math.abs(rotation2.getPosition()-rot2pos)<0.05) {
                c3=0;
                slides.setTargetPosition(0);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(0.6);
                claw2pos=0.25;
                claw2.setPosition(claw2pos);

            }

        }

    }

}