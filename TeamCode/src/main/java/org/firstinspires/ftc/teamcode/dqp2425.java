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

//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlide;

@Config
@TeleOp(name = "dqp2425", group = "TeleOp")

public class dqp2425 extends LinearOpMode{
//    private Drivetrain drivetrain;
//    private OuttakeSlide outtakeSlide;
private DcMotor slides;
    private Servo slides2;
    private Servo rotation;
    private Servo pivot;
    private Servo claw;
    private Servo claw2;
    private Servo rotation2;
    private DcMotor hang; // linear actuator
    private DcMotor winch; //winch
    private Servo swing;

    double pivotpos=0.53;
    double pivotnuetral = 0.53;


    //intake claw
    double clawpos=0.47;
    double clawclose = 0.06;
    double clawopen = 0.45;

    //intake rotation
    double rotpos= 1;
    double rotin = 0.28;
    double rotout = 0.95;
    int slidesnuetral = -110;
    int slidesSpeci = -38;
    int slidesLatchOff = -174;
    int slidesup = -757;

    //outtake claw
    double claw2pos=0.33;
    double claw2close = 0.33;
    double claw2open = 0.1;


    //outtake arm rotation
    double rot2pos=0.7;
    double rot2down = 0.7821;
    double rot2out = 0.12;
    double rot2wall = 0.98;
    double rot2speci = 0.371;

    //intake axon
    double slides2pos=0.4968;
    double slides2out = 0.77;
    double slides2in= 0.4968;

    // winch down position line 249 may have to reverse motor
    int winchDown = -1000;
    int actuatorUp = 1664;
    int actuatorHang = 465;

    // swing up is dpad left, down is dpad right
    double swingup = 0.105;
    double swingdown = 0.003;


    double f = 0;
    int c1=0;
    int c2=0;
    int c3=0;

    int b1=0;

    int a1=0;
    double a2=0;

    boolean start = true;

    @Override
    public void runOpMode() {

        double dampSpeedRatio ;
        double dampTurnRatio ;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL"); //0
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR"); //1
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BL"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BR"); //3
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//        drivetrain = new Drivetrain(hardwareMap);
//        outtakeSlide = new OuttakeSlide(hardwareMap);
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
        winch=hardwareMap.get(DcMotor.class, "hang2");
        swing = hardwareMap.get(Servo.class, "swing"); // control hub port 5

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //*********INIT POSITIONS**********
        claw.setPosition(clawopen);
        rotation.setPosition(0.28);
        rotation2.setPosition(0.7);
        swing.setPosition(swingdown);




        waitForStart();
        while (opModeIsActive()) {
//            drivetrain.Teleop(gamepad1,telemetry);
//            outtakeSlide.Teleop(gamepad1,telemetry);
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
            if(start){
                slides.setTargetPosition(slidesnuetral);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                start=false;
            }

            double y = Range.clip(-gamepad1.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(-gamepad1.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(-gamepad1.right_stick_x, -1, 1);

            //    double arct = 0;

            if(gamepad1.right_bumper){
                dampSpeedRatio = 0.33;
                dampTurnRatio = -0.22;
            }else{
                dampSpeedRatio = 1;
                dampTurnRatio = -0.75;
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

            double idkman= this.gamepad2.left_stick_y;
            slides2pos -= idkman/800;
            if(slides2pos >0.8){
                slides2pos = 0.8;
            }
            if (slides2pos<slides2in){
                slides2pos = slides2in;
            }
            if(gamepad2.b){
                slides2pos=slides2out;
                rotpos=rotout-0.1;
                rotation.setPosition(rotpos);
            }

            if(gamepad1.dpad_left){
                c3=1;
                rot2pos = 0.85;
                rotation2.setPosition(rot2pos);

            }
            if(gamepad1.y){
                c3=2;
            }
            if(c3==1 && swing.getPosition()<swingup){
                swing.setPosition(swing.getPosition()+0.0015);
            }else if(c3==2){
                swing.setPosition(swingup+0.01);
            }
            telemetry.addData("swing position", swing.getPosition());


            slides2.setPosition(slides2pos);

            slides.setPower(1);
            if (gamepad2.right_stick_y > 0) {
                slides.setTargetPosition(slides.getCurrentPosition() +50);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //slides.setPower(gamepad2.right_stick_y );
            }
            if (gamepad2.right_stick_y < 0) {
                slides.setTargetPosition(slides.getCurrentPosition() -50);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //slides.setPower(gamepad2.right_stick_y);
            }
            if(gamepad1.y){
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            telemetry.addData("slides", slides.getCurrentPosition());
            telemetry.addData("hang", hang.getCurrentPosition());



            if (gamepad1.right_trigger>0) {
                hang.setTargetPosition(actuatorUp);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(1);
            }else if (gamepad1.left_trigger>0) {
                hang.setTargetPosition(actuatorHang);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(1);
            }else{
                hang.setPower(0);
                //hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            // will have to comment out b/c running to position

            /*
            if (gamepad1.a) {
                winch.setPower(0.6);
            }
            else if (gamepad1.b) {
                winch.setPower(-0.6);
            }
            else {
                winch.setPower(0);
            }*/


            if (gamepad1.a){//going up
                winch.setTargetPosition(winch.getCurrentPosition()+150);
                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winch.setPower(0.6);
            }

            if (gamepad1.b){//going down
                winch.setTargetPosition(winch.getCurrentPosition()-150);
                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winch.setPower(0.6);
            }

            telemetry.addData("winch position", winch.getCurrentPosition());
//
            if (gamepad2.a) {
                rotpos=rotout;
                rotation.setPosition(rotpos);
            }
            if (gamepad2.dpad_left) {
                rotpos=rotin;
                rotation.setPosition(rotpos);
            }
            if (gamepad2.right_bumper) {

                claw2pos = claw2close;
                claw2.setPosition(claw2pos);
            }
            if (gamepad2.left_bumper) {

                claw2pos=claw2open;
                claw2.setPosition(claw2pos);
            }
            if (gamepad2.x) {
                clawpos=clawclose;//0.08 close
                claw.setPosition(clawpos);
            }
            if (gamepad2.y) {
                clawpos=clawopen;//-0.47
                claw.setPosition(clawpos);
            }
            pivotpos-=gamepad2.left_stick_x*0.017;

            if (pivotpos>1) {
                pivotpos=1;
            }
            if (pivotpos<0){
                pivotpos=0;
            }

            pivot.setPosition(pivotpos);
            rot2pos-=gamepad2.left_trigger/100;
            rot2pos += gamepad2.right_trigger/100;
            if(rot2pos<0){
                rot2pos=0;
            }
            if(rot2pos >1){
                rot2pos =1;
            }



            if (gamepad2.right_trigger>0) {
                if (rot2pos<0.985){
                    rot2pos+=gamepad2.right_trigger/100;
                }
                else{
                    rot2pos=0.985;
                }
//

            }

            if(gamepad2.dpad_right && b1==0){
                rot2pos= rot2wall;
                b1 = 1;
            }
            if(b1==1){
                f+=0.025;
                if(f>=0.5){
                    b1=2;
                }
            }
            if(b1==2){
                f=0;
                slides.setTargetPosition(-3);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                claw2pos = claw2open;
                claw2.setPosition(claw2pos);
                b1=3;
            }
            if(b1==3){
                if(gamepad2.dpad_right){
                    claw2pos = claw2open;
                    claw2.setPosition(claw2pos);
                }else{
                    claw2pos = claw2close;
                    claw2.setPosition(claw2pos);
                    //slides.setTargetPosition(slidesLatchOff);
                    //slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    b1=4;
                }
            }
            if(b1==4){
                f+=0.025;
                if(f>=0.2){
                    b1=5;
                    f=0;
                }
            }
            if(b1==5){
                slides.setTargetPosition(slidesLatchOff);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(Math.abs(slides.getCurrentPosition()-slidesLatchOff)<=10 ){
                    b1=6;
                }

            }
            if(b1==6){
                //if(Math.abs(slides.getCurrentPosition()-slidesLatchOff)<=10 ){
                    rot2pos= rot2speci;
                    rotation2.setPosition(rot2pos);
                //}
                b1=7;
            }
            if(b1==7){
                f+=0.025;
                if(f>=0.5){
                    b1=8;
                    f=0;
                }
            }
            if(b1==8){
                slides.setTargetPosition(slidesSpeci);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                b1=0;
            }

            telemetry.addData("b1", b1);





            rotation2.setPosition(rot2pos);


            // AUTOMATION STUFF
            if (gamepad2.right_stick_button) {
                //transfer()
                slides.setTargetPosition(slidesnuetral);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(0.8);
                claw2pos=claw2open;
                rot2pos=rot2down;
                rotation2.setPosition(rot2pos);
                claw2.setPosition(claw2pos);
                clawpos=clawclose;
                claw.setPosition(clawpos);
                //picked up sample on floor
                a1=1;
            }
            if(a1==1){
                a2+=0.015;
                if(a2>=0.2) {

                    if( (claw.getPosition()-clawpos)<= 0.0001 ){
                        a1=2;
                        rotpos=rotin;
                        rotation.setPosition(rotpos);
                        pivotpos=pivotnuetral;
                        pivot.setPosition(pivotpos);
                        a2=0;
                        //rotating up and pivot nuetral
                    }

                }

            }


            //waiting for the rotation to finish
            if(a1==2){
                a2+=0.015;
                if(a2>=0.2) {
                    a1=3;
                }
            }

            if(a1==2){
                if( (rotation.getPosition()-rotpos)<= 0.00001 ){
                    a1=3;
                    slides2pos=slides2in;
                    slides2.setPosition(slides2pos);
                    //sending slides in
                }
            }

            if(a1==3){
                if ((slides2.getPosition() - slides2pos) <= 0.00001) {
                    a2 += 0.015;
                    if (a2 >= 0.8) {
                        //waiting for slides to arrive
                        claw2pos = claw2close;
                        claw2.setPosition(claw2pos);
                        a1 = 4;
                        a2 = 0;
                        //outtake claw closing up
                    }
                }

            }

            if(a1==4){
                a2+=0.015;
                if(a2>=0.13) {
                    clawpos = clawopen;
                    claw.setPosition(clawpos);
                    a1 = 5;
                    a2=0;
                    //intake claw opening
                }
            }

            if(a1==5){
                a2+=0.015;
                if(a2>=0.3) {
                    //waiting for intake claw to open to slide away
                    slides2pos=0.5;
                    slides2.setPosition(slides2pos);
                    a1 = 0;
                    a2=0;

                }
            }





            if (gamepad2.dpad_up) {
                slides.setTargetPosition(slidesup);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(1);
                c1=1;

            }
            if (c1==1 && Math.abs(slides.getCurrentPosition()+slidesup)<=100) {
                c1=0;
                rot2pos=rot2out;
                rotation2.setPosition(rot2out);
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }



            if (gamepad2.dpad_down) {
                c2=1;
            }
            if(c2 ==1){
                if(gamepad2.dpad_down){
                    claw2pos=claw2open;
                    claw2.setPosition(claw2pos);
                }else{
                    c2=2;
                }
            }
            if (c2==2){
                rot2pos=rot2down;
                rotation2.setPosition(rot2pos);
                slides.setTargetPosition(slidesnuetral);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setPower(1);
                claw2pos=claw2close;
                claw2.setPosition(claw2pos);
                c2=0;
            }


        }



    }

}