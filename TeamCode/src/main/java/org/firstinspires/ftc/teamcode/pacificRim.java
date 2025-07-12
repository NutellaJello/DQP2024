package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlide;

@Config
@TeleOp(name = "pacific rim", group = "TeleOp")

public class pacificRim extends LinearOpMode{
    private Drivetrain drivetrain;
//    private OuttakeSlide outtakeSlide;
    /**
     * outtake slide
     */
    private DcMotor slides;     // outtake slide
    private Servo slides2;      // intake slide
    private Servo rotation;     // intake claw green pitch
    private Servo pivot;        // intake claw red rotation
    private Servo claw;         // intake claw
    private Servo claw2;        // outtake claw
    private Servo rotation2;    // outtake arm
    private DcMotor hang;       // linear actuator, thread bearing
    private DcMotor winch;      // winch: the pulley roller to wind the hook string
    private Servo swing;        // the folded bar that the hook is attached to

    boolean fieldCentric = false; // aiden prefers non fieldCentric and Kyle prefers fieldCentric

    double pivotpos=0.53;
    double pivotnuetral = 0.53;


    //intake claw
    double clawpos=0.47;
    double clawclose = 0.0;
    double clawopen = 0.45;

    //intake rotation
    double rotpos= 1;
    double rotin = 0.28;
    double rotout = 0.95;
    int slidesnuetral = -230;// -235
    int slidesSpeci = -5;
    int slidesLatchOff = -442;
    int slidesup = -890;
    int slidesMiddleBasket = -300;

    //outtake claw
    double claw2pos=0.347;
    double claw2close = 0.53; // 0.523
    double claw2open = 0.35;
//

    //outtake arm rotation
    double rot2pos=0.6;
    double rot2down = 0.7714;
    double rot2out = 0.0;
    double rot2wall = 1;
    double rot2speci = 0.54;//0.371

    //intake axon
    double slides2pos=0.4968;
    double slides2out = 0.77;
    double slides2in= 0.4832;

    // winch down position line 249 may have to reverse motor
    //int winchDown = -1000;
    int actuatorUp = 1664;
    int actuatorHang = 465;

    // swing up is dpad left, down is dpad right
    double swingup = 0.178;
    double swingdown = 0.87;

    double transferTime = 0.53;
    boolean usingJst = false;
    boolean startOuttake = false;
    boolean sampleMode = true;
    boolean lowBar = false;


    double f = 0;
    int c1=0;
    int c2=0;
    int c3=0;

    int b1=0;

    int a1=0;
    double a2=0;

    int d1=0;

    int e1 = 0;
    double e2 = 0;

    boolean start = true;
    boolean gooby = true;




    @Override
    public void runOpMode() {
        // initializes movement motors
        drivetrain = new Drivetrain(hardwareMap);
//        outtakeSlide = new OuttakeSlide(hardwareMap);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        slides=hardwareMap.get(DcMotor.class, "slides");// outtake, EPM 0
        slides2=hardwareMap.get(Servo.class, "slides2");// intake, EPS 0 "AXONMAX"
        rotation=hardwareMap.get(Servo.class, "rotation"); // EPS 3
        pivot=hardwareMap.get(Servo.class, "pivot"); // EPS 2
        claw=hardwareMap.get(Servo.class, "claw"); // EPS 1
        claw2=hardwareMap.get(Servo.class, "claw2"); // EPS 4
        rotation2=hardwareMap.get(Servo.class, "rotation2"); // EPS 5
        hang=hardwareMap.get(DcMotor.class, "hang"); // EPM 2
        winch=hardwareMap.get(DcMotor.class, "hang2"); // EPM 3
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
        rotation2.setPosition(rot2pos);
        swing.setPosition(swingdown);
        imu.resetYaw();

        if(fieldCentric){
            transferTime = 0.56;
        }else{
            transferTime = 0.935;// 0.7
        }

//


        waitForStart();
        while (opModeIsActive()) {
            // all the movement controls.
            drivetrain.Teleop(gamepad1,telemetry, fieldCentric);

            telemetry.addData("Samples: ",sampleMode );
            telemetry.addData("slides2", slides2pos);
            telemetry.addData("claw", clawpos);
            telemetry.addData("pivot", pivotpos);
            telemetry.addData("rotation", rotpos);
            telemetry.addData("claw2", claw2pos);
            telemetry.addData("rotation2", rot2pos);
            telemetry.addData("outtake",slides.getCurrentPosition());
            telemetry.addData("a1", a1);
            telemetry.addData("jst", drivetrain.getIntakePosi()); //208 is home base expansion hub analog input, port 2
            // telemetry.addData("coder1", par0.getPositionAndVelocity().rawPosition);
            //telemetry.addData("coder2", par1.getPositionAndVelocity().rawPosition);
            //telemetry.addData("coder3", perp.getPositionAndVelocity().rawPosition);

            if(gamepad1.dpad_left){
                fieldCentric = !fieldCentric;
                drivetrain.Teleop(gamepad1,telemetry, fieldCentric);
                sleep(400);
            }

            if(gamepad2.back){
                sampleMode = true;
            }
            if(gamepad1.back){
                sampleMode = false;
            }
            /*
            if(gamepad1.a){
                rot2pos=0.4;
                rotation2.setPosition(rot2pos);
            }*/

/*
            if(gamepad1.b){
                rot2pos = 0.468;
                slides.setTargetPosition(-460);
            }
*/

            // just outtake slide initialization, put in initialization code
            if(start){
                slides.setTargetPosition(slidesnuetral);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                start=false;
            }

            if(gamepad2.x){
                claw2.setPosition(claw2open);
                claw2pos = claw2open;
            }
            /*if(gamepad2.left_bumper){
                claw2.setPosition(claw2close);
                claw2pos = claw2close;
            }*/



            double idkman= this.gamepad2.left_stick_y;
            slides2pos -= idkman/500;
            if(slides2pos >0.8){
                slides2pos = 0.8;
            }
            if (slides2pos<slides2in){
                slides2pos = slides2in;
            }
            if(gamepad1.b){
                slides2pos=slides2out;
                rotpos=rotout-0.1;
                rotation.setPosition(rotpos);
                clawpos = clawopen;
                claw.setPosition(clawpos);
                a1=0;
            }

            // gamepad1 controls the chassis, this rotation2 is the outtakeArm, it prevents from collision
            if(gamepad2.y){
                c3=1;
                slides.setTargetPosition(-10);
                rot2pos = rot2wall;
                rotation2.setPosition(rot2pos);
                clawpos = clawclose;
                claw.setPosition(clawpos);
                slides2pos=slides2in+0.02;
                slides2.setPosition(slides2pos);
            }
            if(gamepad1.dpad_right){
                c3=2;
            }
            if(gamepad1.x){
                c3=3;
            }
            if(c3==1 && swing.getPosition()> swingup){

                swing.setPosition(swing.getPosition()-0.008);
            }else if(c3==2) {
                swing.setPosition(0.2);
            }else if (c3==3){
                if(gamepad1.x){
                    swing.setPosition(swing.getPosition()-0.01);
                }else if(gamepad1.left_bumper){
                    swing.setPosition(swing.getPosition()+0.01);
                }
            }
            telemetry.addData("swing position", swing.getPosition());


            slides2.setPosition(slides2pos);

            slides.setPower(1);
            if (gamepad2.right_stick_y > 0 && a1==0) {
                slides.setTargetPosition((int) (slides.getCurrentPosition() + (100 * gamepad2.right_stick_y)));
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //slides.setPower(gamepad2.right_stick_y );
            }
            if (gamepad2.right_stick_y < 0 && a1==0) {
                slides.setTargetPosition((int) (slides.getCurrentPosition() + (100 * gamepad2.right_stick_y)));
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //slides.setPower(gamepad2.right_stick_y);
            }/*
            if(gamepad1.y){
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slides.setPower(0);
                sleep(100);
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }*/
            telemetry.addData("slides", slides.getCurrentPosition());
            telemetry.addData("hang", hang.getCurrentPosition());

            if(gamepad2.x){
                pivotpos = 0.18;
                pivot.setPosition(pivotpos);
            }//e

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
            }

            if(gamepad1.left_stick_button){
                d1=1;
            }
            if(d1==1){
                if(gamepad2.left_stick_button) {
                    clawpos = clawclose;
                    claw.setPosition(clawpos);


                }else{
                    rotpos = 0.75;
                    rotation.setPosition(rotpos);
                    slides2pos = slides2in;
                    slides2.setPosition(slides2pos);
                    d1=0;
                }

            }

           /* if (gamepad2.left_trigger>=0.1){//going up
                winch.setTargetPosition(winch.getCurrentPosition()+150);
                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winch.setPower(gamepad2.left_trigger);
            }*/

            if (gamepad2.left_trigger>=0.1 && gamepad2.y){//going down
                winch.setTargetPosition(winch.getCurrentPosition()-150);
                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winch.setPower(0.6);
            }else if(gamepad2.left_trigger>=0.1 || gamepad2.right_trigger>=0.1){
                winch.setTargetPosition((int) (winch.getCurrentPosition()+200*gamepad2.left_trigger));
                winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winch.setPower(1);
            }

            telemetry.addData("winch position", winch.getCurrentPosition());
//
            if (gamepad1.a) {
                rotpos=rotout;
                rotation.setPosition(rotpos);
            }
            if (gamepad2.dpad_left) {
                rotpos=rotin;
                rotation.setPosition(rotpos);
            }
            if (gamepad2.left_bumper && e1==0) {
                slides2pos= slides2out;
                slides2.setPosition(slides2pos);
                e1=1;
            }
            if(e1 == 1){
                if(!gamepad2.left_bumper){
                    clawpos=clawopen;
                    claw.setPosition(clawpos);
                    slides2pos=slides2in;
                    slides2.setPosition(slides2pos);
                    pivotpos = pivotnuetral;
                    pivot.setPosition(pivotpos);
                    e1=0;
                }
            }
            /*if(e1==2){
                clawpos=clawopen;
                claw.setPosition(clawpos);
                slides2pos=slides2in;
                slides2.setPosition(slides2pos);
                pivotpos = pivotnuetral;
                pivot.setPosition(pivotpos);
                e1=0;
            }*/

            if((gamepad1.right_stick_y>=0.90 || gamepad1.right_stick_y<=0.90) &&a1==0) {
                pivotpos -= gamepad1.right_stick_y * 0.013;
            }

            if (pivotpos>1) {
                pivotpos=1;
            }
            if (pivotpos<0){
                pivotpos=0;
            }

            pivot.setPosition(pivotpos);
            //rot2pos-=gamepad2.left_trigger/100;
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
            }

            if(gamepad2.right_bumper && b1==0){
                rot2pos= rot2wall;
                rotpos = 0.75;
                rotation.setPosition(rotpos);
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
                slides.setTargetPosition(-1);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                claw2pos = claw2open;
                claw2.setPosition(claw2pos);
                b1=3;
                gooby = true;
            }
            if(b1==3){
                if(gamepad2.right_bumper){
                    claw2pos = claw2open-0.15;
                    claw2.setPosition(claw2pos);
                    if(gamepad1.y){
                        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        gooby = false;
                    }

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
                b1=0;
            }
            /* if(b1==7){
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
            }*/

            telemetry.addData("b1", b1);





            rotation2.setPosition(rot2pos);


            // AUTOMATION STUFF
            // this picks up a sample and transfers it
            if (gamepad1.right_stick_button) {
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
                if(a2>=0.05) {

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
                if(a2>=0.30) {//0.18
                    a1=3;
                    a2=0;
                }
            }

            if(a1==3){
                if( (rotation.getPosition()-rotpos)<= 0.00001 ){
                    a1=4;
                    slides2pos=slides2in;
                    slides2.setPosition(slides2pos);
                    pivotpos=pivotnuetral; // 553 to 554 ensures its in the right position
                    pivot.setPosition(pivotpos);
                    //sending slides in
                }
            }

            if(a1==4){
                if(sampleMode){
                    usingJst = true;
                    if(drivetrain.getIntakePosi() >= 210){
                        a2 += 0.015;
                        if (a2 >= transferTime/1.7) {
                            //waiting for slides to arrive
                            claw2pos = claw2close;
                            claw2.setPosition(claw2pos);
                            a1=5;
                            a2=0;
                            //outtake claw closing up
                        }


                    }
                }else {
                    usingJst = false;
                    if ((slides2.getPosition() - slides2pos) <= 0.00001) {
                        a2 += 0.015;
                        if (a2 >= transferTime) {
                            //waiting for slides to arrive
                            claw2pos = claw2close;
                            claw2.setPosition(claw2pos);
                            a1 = 5;
                            a2 = 0;
                            //outtake claw closing up
                        }
                    }
                }
            }

            if(a1==5){
                a2+=0.015;
                if(a2>=0.1) {
                    clawpos = clawopen;
                    claw.setPosition(clawpos);
                    a1=0;
                    a2=0;
                    //intake claw opening
                    if(usingJst){
                        startOuttake = true;

                    }
                }

            }


            if(gamepad2.left_stick_button){
                lowBar = !lowBar;
            }
            telemetry.addData("lowBar: ", lowBar);


            if (gamepad2.dpad_up || startOuttake) {
                if(lowBar){
                    slides.setTargetPosition(slidesMiddleBasket);
                    slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slides.setPower(1);
                    c1=1;
                    startOuttake = false;
                }
                else{
                    slides.setTargetPosition(slidesup);
                    slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slides.setPower(1);
                    c1=1;
                    startOuttake = false;
                }


            }
            if (c1==1 /*&& Math.abs(slides.getCurrentPosition()-slidesup)<=100*/) {
                c1=0;
                rot2pos=rot2out;
                rotation2.setPosition(rot2out);
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }



            if (gamepad1.dpad_down ) {
                c2=1;
            }
            if(c2 ==1){
                if(gamepad1.dpad_down){
                    pivotpos=pivotnuetral;
                    pivot.setPosition(pivotpos); // added 645 and 644 because pivot claw is :(
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
                slides.setPower(1);/*
                claw2pos=claw2close;
                claw2.setPosition(claw2pos);*/
                c2=0;
            }


        }


    }

}