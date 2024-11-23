package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
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
public class RedSideObservation extends LinearOpMode {

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
                            intakeRotation.setPosition(0.989);
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
                            intakeSlides.setPosition(0.5726);
                            return false;
                        }}, new SleepAction(1.4)
            );
        }


        public Action retractPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.702);
                            return false;
                        }}, new SleepAction(1.4)
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
        public Action outtakeOut() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.143);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }


        public Action outRotThrowUp() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.31);
                            return false;
                        }}, new SleepAction(0.6)
            );
        }
        public Action outRotThrowDown() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.31);
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, mirrorPose.apply(new Pose2d(6, -60, Math.toRadians(270))));

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

        // Configure motor and servo settings
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Ensure intake does not move around
        intakeSlides.setPosition(0.69);
        //Hold Spece
        rotation2.setPosition(0.143);
        outtakeClaw.setPosition(0.38);

//        outtakeClaw.setPosition(0.25);

//
        // Autonomous Actions
        TrajectoryActionBuilder tab1 = drive.actionBuilder(mirrorPose.apply(new Pose2d(6,-60,Math.toRadians(270))))
                .setReversed(true)
                .strafeTo(mirrorVector.apply(new Vector2d(0,-29)));
        Action movement1=tab1.build();

        TrajectoryActionBuilder tab2 = drive.actionBuilder(mirrorPose.apply(new Pose2d(0, -29, Math.toRadians(270))))
                .lineToY(35)
                .strafeTo(mirrorVector.apply(new Vector2d(42,-42.2)))
                .waitSeconds(0.1)
                .turn(Math.toRadians(179.5));
        Action movement2 = tab2.build();

        TrajectoryActionBuilder back1=drive.actionBuilder(mirrorPose.apply(new Pose2d(42,-42.2,Math.toRadians(90))))
                .lineToY(50);
        Action b1 = back1.build();
        TrajectoryActionBuilder tab3 = drive.actionBuilder(mirrorPose.apply(new Pose2d(42, -50, Math.toRadians(90))))
                .strafeTo(mirrorVector.apply(new Vector2d(53,-43)), new TranslationalVelConstraint(12.0));
        Action movement3 = tab3.build();

        TrajectoryActionBuilder tab4 = drive.actionBuilder(mirrorPose.apply(new Pose2d(54, -44.5, Math.toRadians(90))))
                .strafeTo(mirrorVector.apply(new Vector2d(46,-46)))
                .waitSeconds(0.1)
                .turn(Math.toRadians(180));
        Action movement4 = tab4.build();

        TrajectoryActionBuilder tab5 = drive.actionBuilder(mirrorPose.apply(new Pose2d(50, -46, Math.toRadians(270))))
                .strafeTo(mirrorVector.apply(new Vector2d(6,-46)))
                .setReversed(true)
                .lineToY(29);
        Action movement5 = tab5.build();

        TrajectoryActionBuilder tab = drive.actionBuilder(mirrorPose.apply(new Pose2d(59, -46, Math.toRadians(90))))
                .splineTo(mirrorVector.apply(new Vector2d(30,-10)),Math.toRadians(0))
                .lineToX(-60)
                .strafeTo(mirrorVector.apply(new Vector2d(60,-40)))
                .strafeTo(mirrorVector.apply(new Vector2d(60,-30)))
                .turn(Math.toRadians(90));

        Action movement = tab5.build();


        TrajectoryActionBuilder firstpickup=drive.actionBuilder(mirrorPose.apply(new Pose2d(42,-50,Math.toRadians(90))))
                .strafeTo(mirrorVector.apply(new Vector2d(36,-39)))
                .turnTo(Math.toRadians(266-180));
        Action pick1 = firstpickup.build();
        TrajectoryActionBuilder back = drive.actionBuilder(mirrorPose.apply(new Pose2d(36, -39, Math.toRadians(270))))
                .strafeTo(mirrorVector.apply(new Vector2d(0, -35)));
        Action goback=back.build();
        Action slightforward=drive.actionBuilder(mirrorPose.apply(new Pose2d(0,-35,Math.toRadians(270))))
                .lineToY(24.37).build();
        Action park=drive.actionBuilder(mirrorPose.apply(new Pose2d(0,-24.37,Math.toRadians(270))))
                .strafeTo(mirrorVector.apply(new Vector2d(50,-55))).build();



        // Use utility methods to create actions
        Action slidesUp = createMotorAction(slides, -2300, 0.8);      // Slides up
        Action slidesPartUp=createMotorAction(slides,-1000,0.8);
        Action slidesPartDown = createMotorAction(slides, -400, 0.3); // Slides partially down
        Action slidesDown = createMotorAction(slides,-5 , 0.6);       // Slides down
        Action slidesDown2 = createMotorAction(slides, -5, 0.6);
        Action slidesUp2 = createMotorAction(slides, -2300, 0.8);
        Action slidesDown3 = createMotorAction(slides, -5, 0.6);
        Action slidesPartUp2 = createMotorAction(slides, -1000, 0.8);
        Action slidesPartDown2=createMotorAction(slides, -400, 0.3);

        ArrayList<Action> pickSampleActions = new ArrayList<>();
        pickSampleActions.add(slidesDown);

        Action pickSample = new ConcurrentAction(pickSampleActions);


        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(outtakeClaw.closeClaw(),
                                slidesPartUp,
                                movement1),
                        slidesPartDown,
                        outtakeClaw.openClaw(),
                        new ParallelAction(slidesDown,
                                movement2),
                        intakeClaw.openClaw(),
                        // outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotDown(),
//                        new SleepAction(1),
                        intakeSlides.moveToPosition(),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        intakeRotation.intakeRotUp(),
                        intakeSlides.retractPosition(),
                        outtakeClaw.closeClaw(),
//                        new SleepAction(1),
                        intakeClaw.openClaw(),
                        new ParallelAction(
                                b1,
                                outtakeRotation.outRotThrowUp()),
                        // new SleepAction(1),
                        new ParallelAction(
                                outtakeRotation.outRotThrowDown(),
                                outtakeClaw.openClaw()
                        ),
                        new ParallelAction(
                                pick1,
                                outtakeRotation.outtakeRotDown()),
                        intakeSlides.moveToPosition(),
                        intakeRotation.intakeRotDown(),
                        new ParallelAction(goback, new SequentialAction(
                                intakeClaw.closeClaw(),
                                intakeRotation.intakeRotUp(),
                                intakeSlides.retractPosition(),
                                outtakeClaw.closeClaw(), intakeClaw.openClaw(), slidesPartUp2)),
                        new ParallelAction(slightforward, outtakeRotation.outtakeOut()),
                        slidesPartDown2,
                        outtakeClaw.openClaw(),
                        new ParallelAction(slidesDown2, park)






//                        movement3,
//                        outtakeRotation.outtakeRotDown(),
//                         intakeSlides.moveToPosition(),
//                         intakeRotation.intakeRotDown(),
//                         intakeClaw.closeClaw(),
//                         intakeRotation.intakeRotUp(),
//                         intakeSlides.retractPosition(),
////                        new SleepAction(1),
//                        outtakeClaw.closeClaw(),
//                         intakeClaw.openClaw(),
//                        outtakeRotation.outRotThrowUp(),
//                        new ParallelAction(
//                                outtakeRotation.outRotThrowDown(),
//                                outtakeClaw.openClaw()
//                        )
//                        movement4,
//                        movement5




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
