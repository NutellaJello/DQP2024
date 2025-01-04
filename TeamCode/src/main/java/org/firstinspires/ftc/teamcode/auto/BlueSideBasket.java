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
                            intakeClaw.setPosition(0.06);
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
                            intakeRotation.setPosition(0.95);
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
                            intakeSlides.setPosition(0.64);
                            return false;
                        }}, new SleepAction(0.4)
            );
        }


        public Action retractPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.5);
                            return false;
                        }}, new SleepAction(0.4)
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
                            outtakeClaw.setPosition(0.1);
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
        public Action outtakeRotFinal() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.55);
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

        public Action outtakeRotBasket() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.06);
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
        rotation = hardwareMap.get(Servo.class, "rotation");
        pivot = hardwareMap.get(Servo.class, "pivot");
          claw = hardwareMap.get(Servo.class, "claw");
          claw2 = hardwareMap.get(Servo.class, "claw2");
        rotation2 = hardwareMap.get(Servo.class, "rotation2");
         rotation = hardwareMap.get(Servo.class, "rotation");
        rotation = hardwareMap.get(Servo.class, "rotation");


        // Configure motor and servo settings
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Ensure intake does not move around
        intakeSlides.setPosition(0.4968);
        claw.setPosition(0.45);
        //Hold Spece
        rotation2.setPosition(0.64);
        outtakeClaw.setPosition(0.347);

        // Autonomous Actions

        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-16,-60,Math.toRadians(90)))
                .splineTo(new Vector2d(-5,-29), Math.toRadians(90));


        Action movement=tab.build();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-5, -29, Math.toRadians(90)))
                .lineToY(-44)
                .strafeTo(new Vector2d(-44,-42.4));

        Action movement1 = tab1.build();
//
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-45.5, -42.4, Math.toRadians(90)))
                .strafeTo(new Vector2d(-58.5,-55.5))
                .turnTo(45);
        Action movement2 = tab2.build();

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-58.5, -55.5, Math.toRadians(45)))
                .splineTo(new Vector2d(-53,-42.4), Math.toRadians(90));
        Action movement3 = tab3.build();

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-55.5,-42.4, Math.toRadians(90)))
                .strafeTo(new Vector2d(-58.5,-55.5))
                .turnTo(45);
        Action movement5 = tab5.build();

        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-58.5,-55.5, Math.toRadians(45)))
                .turnTo(90)
                .splineTo(new Vector2d(-62,-42.4), Math.toRadians(90));
        Action movement6 = tab6.build();

        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-62,-42.4, Math.toRadians(90)))
                .lineToY(-55);
        Action movement7 = tab7.build();
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(-62,-55, Math.toRadians(90)))
                .splineTo(new Vector2d(-30,0), Math.toRadians(0));
        Action movement8 = tab8.build();






        // Use utility methods to create actions
        Action slidesUp = createMotorAction(slides, -860, 0.9);      // Slides up
        Action slidesSpecUp = createMotorAction(slides, -640, 0.8);
        Action slidesDown = createMotorAction(slides,-105 , 0.9);       // Slides down
        Action slidesPartiallyDown = createMotorAction(slides, -240, 0.8);
        Action slidesDown2 = createMotorAction(slides,-105 , 0.9);
        Action slidesUp2 = createMotorAction(slides, -860, 0.9);
        Action slidesDown3 = createMotorAction(slides,-105 , 0.9);
        Action slidesUp3 = createMotorAction(slides, -850, 0.8);
        Action slidesDown4 = createMotorAction(slides,-5 , 0.9);

        ArrayList<Action> pickSampleActions = new ArrayList<>();
        pickSampleActions.add(slidesDown);

        Action pickSample = new ConcurrentAction(pickSampleActions);


        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(movement, slidesSpecUp),
                        slidesPartiallyDown,
                        outtakeClaw.openClaw(),
                        new ParallelAction(movement1,
                                slidesDown,
                                outtakeRotation.outtakeRotTransfer()),
                        intakeSlides.moveToPosition(),
                        intakeRotation.intakeRotDown(),

                        intakeClaw.closeClaw(),
                        intakeRotation.intakeRotUp(),
                        intakeSlides.retractPosition(),
                        outtakeClaw.closeClaw(),
                        intakeClaw.openClaw(),
                        new ParallelAction(movement2,
                                slidesUp),
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        new ParallelAction(movement3, new SequentialAction(
                                outtakeRotation.outtakeRotTransfer(),
                                slidesDown2
                        )),
                        intakeSlides.moveToPosition(),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        intakeRotation.intakeRotUp(),
                        intakeSlides.retractPosition(),
                        outtakeClaw.closeClaw(),
                        intakeClaw.openClaw(),
                        new ParallelAction(movement5, new SequentialAction(
                                slidesUp2,
                                outtakeRotation.outtakeRotBasket()
                        )),
                        outtakeClaw.openClaw(),
                        new ParallelAction(movement6, new SequentialAction(
                                outtakeRotation.outtakeRotTransfer(),
                                slidesDown3
                        )),
                        intakeSlides.moveToPosition(),
                        intakeRotation.intakeRotDown(),
                        new ParallelAction(
                                intakeClaw.closeClaw(),
                                intakeRotation.intakeRotUp(),
                                intakeSlides.retractPosition()
                        ),
                        outtakeClaw.closeClaw(),
                        intakeClaw.openClaw(),
                        new ParallelAction(movement7, new SequentialAction(
                                slidesUp3,
                                outtakeRotation.outtakeRotBasket()
                        )),
                        outtakeClaw.openClaw(),
                        new ParallelAction(movement8, new SequentialAction(
                                outtakeRotation.outtakeRotFinal(),
                                slidesDown4
                        ))


//


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