package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeOuttakeSys;

@Config
@TeleOp(name = "dqp2425", group = "TeleOp")

public class dqp2425_new2 extends LinearOpMode{
    private Drivetrain drivetrain;
    private IntakeOuttakeSys intakeOuttakeSys;

    private DcMotor hang;       // linear actuator, thread bearing
    private DcMotor winch;      // winch: the pulley roller to wind the hook string
    private Servo swing;        // the folded bar that the hook is attached to

    boolean fieldCentric = false;

    // winch down position line 249 may have to reverse motor
    //int winchDown = -1000;
    int actuatorUp = 1664;
    int actuatorHang = 465;

    // swing up is dpad left, down is dpad right
    double swingup = 0.175; // touch bar is 0.23
    double swingdown = 0.9;

    int c3=0;

    @Override
    public void runOpMode() {
        // initializes movement motors
        drivetrain = new Drivetrain(hardwareMap);
        intakeOuttakeSys = new IntakeOuttakeSys(hardwareMap);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        hang=hardwareMap.get(DcMotor.class, "hang"); // EPM 1
        winch=hardwareMap.get(DcMotor.class, "hang2");
        swing = hardwareMap.get(Servo.class, "swing"); // control hub port 5

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //*********INIT POSITIONS**********
        intakeOuttakeSys.Initialize();
        swing.setPosition(swingdown);
        imu.resetYaw();


        waitForStart();
        while (opModeIsActive()) {
            // all the movement controls.
            drivetrain.Teleop(gamepad1,telemetry, fieldCentric);
            intakeOuttakeSys.Teleop(gamepad1, gamepad2, telemetry);

            telemetry.update();

            // gamepad1 controls the chassis, this rotation2 is the outtakeArm, it prevents from collision
            if(gamepad1.dpad_left){
                c3=1;
                intakeOuttakeSys.HangPreventCollision();

            }
            if(gamepad1.dpad_right){
                c3=2;
            }
            if(gamepad1.x){
                c3=3;
            }
            if(c3==1 && swing.getPosition()> swingup){
                swing.setPosition(swing.getPosition()-0.0015);
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
            }

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

        }


    }

}