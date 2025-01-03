package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

@Config
@Autonomous
public class BlueSideBasket extends LinearOpMode {

    // Declare motors and servos
    private DcMotor slides;
    private Servo slides2, rotation, pivot, claw, claw2, rotation2;

    public class intakeClaw {
        private Servo intakeClaw;

        public intakeClaw(HardwareMap hardwareMap) {
            intakeClaw = hardwareMap.get(Servo.class, "claw");
        }
        public Action closeClaw() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeClaw.setPosition(0.08);
                            return false;
                        }}, new SleepAction(0.2)
            );
        }
        public Action openClaw() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeClaw.setPosition(0.45);
                            return false;
                        }}, new SleepAction(0.2)
            );
        }

    }

    public class intakeRotation {
        private Servo intakeRotation;

        public intakeRotation(HardwareMap hardwareMap) {
            intakeRotation = hardwareMap.get(Servo.class, "rotation");
        }
        public Action intakeRotDown() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeRotation.setPosition(0.97);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }

        public Action intakeRotUp() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeRotation.setPosition(0.28);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }
        public void setPosition(double value){
            intakeRotation.setPosition(value);
        }
    }
    public class intakeSlides {
        private Servo intakeSlides;

        public intakeSlides(HardwareMap hardwareMap) {
            intakeSlides = hardwareMap.get(Servo.class, "slides2");
        }

        public Action moveToPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.57);
                            return false;
                        }}, new SleepAction(1.3)
            );
        }


        public Action retractPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.702);
                            return false;
                        }}, new SleepAction(1.3)
            );
        }

        public void setPosition(double value){
            intakeSlides.setPosition(value);
        }
    }

    public class outtakeClaw {
        private Servo outtakeClaw;

        public outtakeClaw(HardwareMap hardwareMap) {
            outtakeClaw = hardwareMap.get(Servo.class, "claw2");
        }
        public Action closeClaw() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeClaw.setPosition(0.38);
                            return false;
                        }
                    }, new SleepAction(0.2)
            );
        }

        public Action openClaw() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeClaw.setPosition(0);
                            return false;
                        }}, new SleepAction(0.2)
            );
        }
        public void setPosition(double value){
            outtakeClaw.setPosition(value);
        }
    }

    public class outtakeRotation {
        private Servo outtakeRotation;

        public outtakeRotation(HardwareMap hardwareMap) {
            outtakeRotation = hardwareMap.get(Servo.class, "rotation2");
        }
        public Action outtakeRotDown() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.97);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }

        public Action outtakeRotUp() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.25);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }
        public void setPosition(double value){
            outtakeRotation.setPosition(value);
        }
    }




    @Override
    public void runOpMode() {
        // Initialize drivetrain and mechanisms
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16, -60, Math.toRadians(90)));


        intakeClaw intakeClaw = new intakeClaw(hardwareMap);
        intakeRotation intakeRotation = new intakeRotation(hardwareMap);
        intakeSlides intakeSlides = new intakeSlides(hardwareMap);

        outtakeClaw outtakeClaw = new outtakeClaw(hardwareMap);
        outtakeRotation outtakeRotation = new outtakeRotation(hardwareMap);

        slides = hardwareMap.get(DcMotor.class, "slides");
        slides2 = hardwareMap.get(Servo.class, "slides2");
        //rotation = hardwareMap.get(Servo.class, "rotation");
        pivot = hardwareMap.get(Servo.class, "pivot");
        //  claw = hardwareMap.get(Servo.class, "claw");
        //  claw2 = hardwareMap.get(Servo.class, "claw2");
        rotation2 = hardwareMap.get(Servo.class, "rotation2");
        // rotation = hardwareMap.get(Servo.class, "rotation");
        rotation = hardwareMap.get(Servo.class, "rotation");


        // Configure motor and servo settings
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Ensure intake does not move around
//        intakeSlides.setPosition(0.69);
        //Hold Spece
//        rotation2.setPosition(0.143);
//        outtakeClaw.setPosition(0.38);

        // Autonomous Actions

        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-16,-60,Math.toRadians(90)))
                .setReversed(true)
                .strafeTo(new Vector2d(-10,-59))
                .waitSeconds(0.2)
                .lineToY(-29);

        Action movement=tab.build();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-10, -29, Math.toRadians(90)))
                .lineToY(-42.4)
                .strafeTo(new Vector2d(-46.5,-42.4))
                .waitSeconds(0.1)
                .turn(Math.toRadians(179.5));
        Action movement1 = tab1.build();
//
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-45.5, -42.4, Math.toRadians(90)))
                .strafeTo(mirrorVector.apply(new Vector2d(-58.5, -55.5))).turnTo(Math.toRadians(45));
        Action movement3 = tab3.build();

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-58.5, -55.5, Math.toRadians(45)))
                .splineTo(new Vector2d(-55.5,-42.4), Math.toRadians(90));
        Action movement2 = tab2.build();

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-55.5,-42.4, Math.toRadians(90)))
                .strafeTo(mirrorVector.apply(new Vector2d(-58.5,-55.5))).turnTo(Math.toRadians(45));
        Action movement5 = tab5.build();





        // Use utility methods to create actions
        Action slidesUp = createMotorAction(slides, -2300, 0.8);      // Slides up
        Action slidesPartUp=createMotorAction(slides,-1100,0.8);
        Action slidesPartDown = createMotorAction(slides, -400, 0.2); // Slides partially down
        Action slidesDown = createMotorAction(slides,-5 , 0.8);       // Slides down
        Action slidesDown2 = createMotorAction(slides, -5, 0.8);
        Action slidesUp2 = createMotorAction(slides, -2300, 0.8);
        Action slidesDown3 = createMotorAction(slides, -5, 0.8);
        ArrayList<Action> pickSampleActions = new ArrayList<>();
        pickSampleActions.add(slidesDown);

        Action pickSample = new ConcurrentAction(pickSampleActions);


        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
//                        outtakeClaw.closeClaw(),
//                        slidesPartUp,
                        movement,
//                        slidesPartDown,
//                        outtakeClaw.openClaw(),
//                        slidesDown,
                        movement1
//                        intakeClaw.openClaw(),
//                        outtakeClaw.openClaw(),
//                        outtakeRotation.outtakeRotDown(),
//                        intakeSlides.moveToPosition(),
//                        intakeRotation.intakeRotDown(),
//                        intakeClaw.closeClaw(),
//                        intakeRotation.intakeRotUp(),
//                        intakeSlides.retractPosition(),
//                        outtakeClaw.closeClaw(),
//                        intakeClaw.openClaw(),
//                        new ParallelAction(movement3, new SequentialAction(
//                                slidesUp,
//                                outtakeRotation.outtakeRotUp()
//                        )),
//                        outtakeClaw.openClaw(),
//                        new ParallelAction(movement2, new SequentialAction(
//                                outtakeRotation.outtakeRotDown(),
//                                slidesDown2
//                        )),
//                        intakeSlides.moveToPosition(),
//                        intakeRotation.intakeRotDown(),
//                        intakeClaw.closeClaw(),
//                        intakeRotation.intakeRotUp(),
//                        intakeSlides.retractPosition(),
//                        outtakeClaw.closeClaw(),
//                        intakeClaw.openClaw(),
//                        new ParallelAction(movement5, new SequentialAction(
//                                slidesUp2,
//                                outtakeRotation.outtakeRotUp()
//                        )),
//                        outtakeClaw.openClaw(),
//                        outtakeRotation.outtakeRotDown(),
//                        slidesDown3
                        // old code end here

//                        movement5,
//                        intakeSlides.moveToPosition(),
//                        intakeRotation.intakeRotDown(),
//                        intakeClaw.closeClaw(),
//                        intakeRotation.intakeRotUp(),
//                        intakeSlides.retractPosition(),
//                        outtakeClaw.closeClaw(),
//                        intakeClaw.openClaw(),
//                        slidesUp,
//                        outtakeRotation.outtakeRotUp(),
//                        outtakeClaw.openClaw(),
//                        outtakeRotation.outtakeRotDown(),
//                        slidesDown


//                        intakeSlides.moveToPosition(), new SleepAction(4),
//                        intakeRotation.intakeRotDown(), new SleepAction(4),
//                        intakeClaw.closeClaw(), new SleepAction(4),
//                        intakeRotation.intakeRotUp(), new SleepAction(4),
//                        intakeSlides.retractPosition()
                        // transfer to outtake claw
                        //movement3
                        // slidesUp
                        // rotate2 0.173
                        // open claw
                        // rotate2Base
                        // slidesDown
                        // rotate to heading 90, check MeepMeep


                )
        );
    }

    /**
     * Utility method to create an action for a motor to move to a target position.
     */
    /**
     * Utility method to create an action for a motor to move to a target position.
     */
    private Action createMotorAction(DcMotor motor, int targetPosition, double power) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setTargetPosition(targetPosition);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(power);
                    initialized = true; // Ensure this setup runs only once
                }

                // Check if motor has reached its target position
                boolean isBusy = motor.isBusy();
                packet.put("Motor Busy", isBusy);
                packet.put("Motor Position", motor.getCurrentPosition());

                // Return true while the motor is still busy
                if (isBusy) {
                    return true;
                } else {
                    motor.setPower(0); // Ensure the motor stops when done
                    return false; // Action is complete
                }
            }
        };
    }

    Function<Pose2d,Pose2d> mirrorPose = pose-> new Pose2d( -pose.position.x, -pose.position.y, pose.heading.toDouble()-Math.PI);
    Function<Vector2d,Vector2d> mirrorVector = vector-> new Vector2d( -vector.x, -vector.y);

    /**
     * Combining multiple Utility methods into one action.
     */
    public class ConcurrentAction implements Action {
        private final List<Action> actions;

        public ConcurrentAction(List<Action> actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean anyRunning = false;
            for (Action action : actions) {
                if (action.run(packet)) {
                    anyRunning = true; // At least one action is still running
                }
            }
            return anyRunning; // Continue running until all actions are complete
        }
    }

}