package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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
import java.util.Arrays;
import java.util.List;

@Config
@Autonomous
public class BlueSideObservation extends LinearOpMode {
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
                            intakeClaw.setPosition(0.05);
                            return false;
                        }}, new SleepAction(0.1)
            );
        }
        public Action openClaw() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeClaw.setPosition(0.45);
                            return false;
                        }}, new SleepAction(0.1)
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
                            intakeRotation.setPosition(0.96);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }

        public Action intakeRotPartialUp() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeRotation.setPosition(0.85);
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
                            intakeSlides.setPosition(0.685);
                            return false;
                        }}, new SleepAction(0.2)
            );
        }
        public Action moveToThirdSample() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.74);
                            return false;
                        }}, new SleepAction(0.2)
            );
        }


        public Action retractPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.49);
                            return false;
                        }}, new SleepAction(0.2)
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
                            outtakeClaw.setPosition(0.347);
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
                            outtakeClaw.setPosition(0.13);
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
        public Action outtakeRotSpec() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.5);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }
        public Action outtakeRotTransfer() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.7821);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }
        public Action outtakeRotWall() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.98);
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(6, -60, Math.toRadians(90)));

        intakeClaw intakeClaw = new intakeClaw(hardwareMap);
        intakeRotation intakeRotation = new intakeRotation(hardwareMap);
        intakeSlides intakeSlides = new intakeSlides(hardwareMap);

        outtakeClaw outtakeClaw = new outtakeClaw(hardwareMap);
        outtakeRotation outtakeRotation = new outtakeRotation(hardwareMap);

        slides = hardwareMap.get(DcMotor.class, "slides");
        slides2 = hardwareMap.get(Servo.class, "slides2");
        rotation = hardwareMap.get(Servo.class, "rotation");
        pivot = hardwareMap.get(Servo.class, "pivot");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        rotation2 = hardwareMap.get(Servo.class, "rotation2");

        // Configure motor and servo settings
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Ensure intake does not move around
        intakeSlides.setPosition(0.4968);
        claw.setPosition(0.45);
        //Hold Spece
        rotation2.setPosition(0.66);
        outtakeClaw.setPosition(0.347);
        // Define custom velocity and acceleration constraints


        int velocity = 100;
        // Autonomous Actions
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(6,-60,Math.toRadians(90)))
                .strafeTo(new Vector2d(-6,-28), new TranslationalVelConstraint(velocity));
        Action movement1=tab1.build();

// actions for tranfering and strafing
        TrajectoryActionBuilder spec1 = drive.actionBuilder(new Pose2d(-7, -28, Math.toRadians(90)))
                .lineToY(-30,new TranslationalVelConstraint(velocity))
                .splineToConstantHeading(new Vector2d(41.75,-47), Math.toRadians(90), new TranslationalVelConstraint(velocity));
        Action firstSample = spec1.build();

        TrajectoryActionBuilder spec2 = drive.actionBuilder(new Pose2d(41.75, -47, Math.toRadians(90)))
                .strafeTo(new Vector2d(55,-47), new TranslationalVelConstraint(velocity));
        Action secondSample = spec2.build();

        TrajectoryActionBuilder spec3 = drive.actionBuilder(new Pose2d(55, -47, Math.toRadians(90)))
                .strafeTo(new Vector2d(46,-47), new TranslationalVelConstraint(velocity))
                .turnTo(Math.toRadians(53));
        Action thirdSample = spec3.build();

        TrajectoryActionBuilder spec3p2 = drive.actionBuilder(new Pose2d(46, -47, Math.toRadians(57)))
                .turnTo(Math.toRadians(90));
        Action thirdSampleTransfer = spec3p2.build();

        TrajectoryActionBuilder test = drive.actionBuilder(new Pose2d(46, -47, Math.toRadians(90)))
                .strafeTo(new Vector2d(28.75,-38), new TranslationalVelConstraint(velocity))
                .strafeTo(new Vector2d(28.75,-63), new TranslationalVelConstraint(velocity));
        Action pickSpecimenWall = test.build();

        TrajectoryActionBuilder toObs = drive.actionBuilder(new Pose2d(-28.75, -63, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-10,-30), Math.toRadians(90), new TranslationalVelConstraint(velocity));
        Action moveToObservation = toObs.build();



        // Use utility methods to create actions
        Action slidesSpecUp=createMotorAction(slides,-660,1);
        Action slidesPartDown = createMotorAction(slides, -290, 1); // Slides partially down
        Action slidesDown = createMotorAction(slides,-5 , 1);       // Slides down

        Action slidesPickSpec = createMotorAction(slides,-200,1);
        Action slidesFirstSpec =createMotorAction(slides,-400,1);

        Action slidesHang =createMotorAction(slides,-680,1);

        Action slidesSpecUp2=createMotorAction(slides,-680,0.9);
        Action slidesPartDown2 = createMotorAction(slides, -290, 0.9);
        Action slidesDown2 = createMotorAction(slides,-5 , 0.9);

        Action slidesTransfer = createMotorAction(slides,-105 , 1);

        ArrayList<Action> pickSampleActions = new ArrayList<>();
        pickSampleActions.add(slidesDown);

        Action pickSample = new ConcurrentAction(pickSampleActions);


        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(movement1,slidesSpecUp),
                        slidesPartDown,
                        outtakeClaw.openClaw(),
                        //first sample
                        new ParallelAction(firstSample,slidesTransfer, outtakeRotation.outtakeRotTransfer()),
                        intakeSlides.moveToPosition(),
                        intakeRotation.intakeRotDown(),
                        new ParallelAction(
                                new SequentialAction(intakeClaw.closeClaw(),new SleepAction(0.1),
                                intakeRotation.intakeRotUp()),
                                intakeSlides.retractPosition()
                        ),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.4),
                        intakeClaw.openClaw(),
                        outtakeRotation.outtakeRotWall(),
                        outtakeClaw.openClaw(),
                        //second sample
                        new ParallelAction(new SequentialAction(outtakeRotation.outtakeRotTransfer(),
                                intakeSlides.moveToPosition()),
                                secondSample),
                        intakeRotation.intakeRotDown(),
                        new ParallelAction(
                                intakeClaw.closeClaw(),
                                intakeRotation.intakeRotUp(),
                                intakeSlides.retractPosition()
                        ),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.2),
                        intakeClaw.openClaw(),
                        outtakeRotation.outtakeRotWall(),
                        outtakeClaw.openClaw(),
                        //third sample
                        new ParallelAction(new SequentialAction(outtakeRotation.outtakeRotTransfer()
                                ,intakeSlides.moveToThirdSample()),
                                thirdSample),
                        intakeRotation.intakeRotDown(),
                        new SleepAction(0.2),
                        new ParallelAction(
                                intakeClaw.closeClaw(),
                                intakeSlides.retractPosition(),
                                intakeRotation.intakeRotUp()
                        ),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.2),
                        intakeClaw.openClaw(),
                        thirdSampleTransfer,
                        outtakeRotation.outtakeRotWall(),
                        outtakeClaw.openClaw(),
                        new ParallelAction(pickSpecimenWall, slidesDown),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.2),
                        slidesPickSpec,
                        new ParallelAction( new SequentialAction(slidesFirstSpec,
                                outtakeRotation.outtakeRotSpec()),
                                moveToObservation),
                        slidesHang,
                        outtakeClaw.openClaw()



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
