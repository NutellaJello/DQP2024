package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.List;
import java.util.function.Function;

@Config
@Autonomous(name = "Basket 4")
public class BlueSide4Samples extends LinearOpMode {

    // Declare motors and servos
    private DcMotor slides;
    private Servo slides2, rotation, pivot, claw, claw2, rotation2, swing;

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
                            intakeClaw.setPosition(0.01);
                            return false;
                        }}, new SleepAction(0.15)
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
                        }}, new SleepAction(0.5)
            );
        }

        public Action intakeRotUp() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeRotation.setPosition(0.28);
                            return false;
                        }}, new SleepAction(0.5)
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
                            intakeSlides.setPosition(0.61);
                            return false;
                        }}, new SleepAction(0.2)
            );
        }


        public Action retractPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.594); // 0.49
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
                            outtakeClaw.setPosition(0.523);
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
                            outtakeClaw.setPosition(0.34);
                            return false;
                        }}, new SleepAction(0.3)
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
                            outtakeRotation.setPosition(0.65); // 0.761 change this so that the transfer happens outside of fully retract
                            return false;
                        }}, new SleepAction(0.3)
            );
        }

        public Action outtakeRotBasket() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.0);
                            return false;
                        }}, new SleepAction(0.75)
            );
        }
        public void setPosition(double value){
            outtakeRotation.setPosition(value);
        }
    }

    public class intakePivot {
        private Servo pivot;

        public intakePivot(HardwareMap hardwareMap) {
            pivot = hardwareMap.get(Servo.class, "pivot");
        }
        public Action resetPivot() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            pivot.setPosition(0.53);
                            return false;
                        }
                    }, new SleepAction(0.2)
            );
        }

        public void setPosition(double value){
            pivot.setPosition(value);
        }
    }
//


    @Override
    public void runOpMode() {
        // Initialize drivetrain and mechanisms
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-37, -60, Math.toRadians(90)));


        intakeClaw intakeClaw = new intakeClaw(hardwareMap);
        intakeRotation intakeRotation = new intakeRotation(hardwareMap);
        intakeSlides intakeSlides = new intakeSlides(hardwareMap);
        intakePivot intakePivot = new intakePivot(hardwareMap);

        outtakeClaw outtakeClaw = new outtakeClaw(hardwareMap);
        outtakeRotation outtakeRotation = new outtakeRotation(hardwareMap);

        //ColorDetectionPipeline pipeline = new ColorDetectionPipeline();
        //webcam.setPipeline(pipeline);







        slides = hardwareMap.get(DcMotor.class, "slides");
        slides2 = hardwareMap.get(Servo.class, "slides2");
        rotation = hardwareMap.get(Servo.class, "rotation");

        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        rotation2 = hardwareMap.get(Servo.class, "rotation2");
        rotation = hardwareMap.get(Servo.class, "rotation");
        rotation = hardwareMap.get(Servo.class, "rotation");
        swing = hardwareMap.get(Servo.class, "swing"); // control hub port 5



        // Configure motor and servo settings
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakePivot.setPosition(0.53);
//
        //Ensure intake does not move around
        intakeSlides.setPosition(0.4968);
        claw.setPosition(0.45);
        swing.setPosition(0.93);

        //Hold Spece
        rotation2.setPosition(0.65);
        outtakeClaw.setPosition(0.525);

        // Autonomous Actions

        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-37,-60,Math.toRadians(90)))
                .strafeTo(new Vector2d(-57.5,-48));
        Action outtake1 = tab.build();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-57.5, -48, Math.toRadians(90)))
                .strafeTo(new Vector2d(-42.5,-41.9));
        Action intake2 = tab1.build();

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-42.5, -41.9, Math.toRadians(90)))
                .strafeTo(new Vector2d(-58.5,-48));
        Action outtake2 = tab2.build();


        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-58.5, -48, Math.toRadians(90)))
                .strafeTo(new Vector2d(-51,-42));
        Action intake3 = tab3.build();

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-51,-42, Math.toRadians(90)))
                .strafeTo(new Vector2d(-59,-48));
        Action outtake3 = tab5.build();

        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-59,-48, Math.toRadians(90)))
                .strafeTo(new Vector2d(-63,-41));
        Action intake4 = tab6.build();

        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-63,-41, Math.toRadians(90)))
                .strafeTo(new Vector2d(-58.5,-48));
        Action outtake4 = tab7.build();

        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(-58.5,-48, Math.toRadians(90)))
                .splineTo(new Vector2d(-23,-8), Math.toRadians(0));
        Action end = tab8.build();




////

        // Use utility methods to create actions
        Action slidesUp = createStayingMotorAction(slides, -890, 1,2);      // Slides up
        Action slidesSpecUp = createMotorAction(slides, -650, 2);
        Action slidesDown = createMotorAction(slides,-114 , 0.9,2);       // Slides down

        Action slidesUp2 = createStayingMotorAction(slides, -890, 1,2);
        Action slidesDown2 = createMotorAction(slides,-112 , 0.9,2);
        Action slidesUp3 = createStayingMotorAction(slides, -890, 1,2);
        // have to add a pause because the transfer on the last one is weird.
        Action slidesPartiallyDown = createMotorAction(slides, -300, 0.9,2);
        Action slidesDown3 = createMotorAction(slides,-108 , 0.9,1);
        Action slidesUp4 = createStayingMotorAction(slides, -890, 1,2);
        Action slidesDownFinal = createMotorAction(slides,5 , 1,2);




        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        //move to basket\
                        new ParallelAction(outtake1,slidesUp),

                        //outtake
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotTransfer(),

                        //2nd sample
                        new ParallelAction(intake2, slidesDown),

                        //intake
                        new ParallelAction(intakeSlides.moveToPosition(), intakePivot.resetPivot()),
                        new SleepAction(0.3),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),



                        //to basket
                        new ParallelAction(
                                outtake2,
                                new SequentialAction(new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                                        outtakeClaw.closeClaw(),
                                        new SleepAction(0.3),
                                        intakeClaw.openClaw(),
                                        slidesUp2)),

                        //outtake
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotTransfer(),

                        //3rd sample
                        new ParallelAction(intake3, slidesDown2),

                        //intake
                        new ParallelAction(intakeSlides.moveToPosition(), intakePivot.resetPivot()),
                        new SleepAction(0.3),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
//                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
//                        outtakeClaw.closeClaw(),
//                        new SleepAction(0.4),
//                        intakeClaw.openClaw(),

                        //to basket
                        new ParallelAction(
                                outtake3,
                                new SequentialAction(new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                                        new SleepAction(0.1),
                                        outtakeClaw.closeClaw(),
                                        new SleepAction(0.4),
                                        intakeClaw.openClaw(),
                                        slidesUp3)),

                        //outtake
                        outtakeRotation.outtakeRotBasket(),
                        new SleepAction(0.1),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotTransfer(),

                        //4th sample
                        new ParallelAction(intake4, slidesPartiallyDown),

                        //intake
                        new ParallelAction(intakeSlides.moveToPosition(), intakePivot.resetPivot()),
                        new SleepAction(0.3),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
//                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
//                        outtakeClaw.closeClaw(),
//                        new SleepAction(0.4),
//                        intakeClaw.openClaw(),

                        //to basket

                        new ParallelAction(
                                outtake4,
                                new SequentialAction(new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                                        new SleepAction(0.1),
                                        slidesDown3,
                                        new SleepAction(0.2),
                                        outtakeClaw.closeClaw(),
                                        new SleepAction(0.3),
                                        intakeClaw.openClaw(),
                                        slidesUp4)),

                        //outtake
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotFinal(),

                        new ParallelAction(end, slidesDownFinal)

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
                if (!initialized) {// Use brake mode
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Use brake mode
                    motor.setTargetPosition(targetPosition);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(power);
                    initialized = true; // Ensure this setup runs only once
                }

                // Check if motor has reached its target position
                boolean isBusy = motor.isBusy();
/*                packet.put("Motor Busy", isBusy);
                packet.put("Motor Position", motor.getCurrentPosition());*/

                // Return true while the motor is still busy
                if (isBusy) {
                    return true;
                } else {
                    motor.setPower(0.01); // Ensure the motor stops when done
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    return false; // Action is complete
                }
            }
        };
    }
    private Action createMotorAction(DcMotor motor, int targetPosition, double power, int tolerance) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Initialization: Set motor target and mode
                if (!initialized) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Use brake mode
                    motor.setTargetPosition(targetPosition);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(power);
                    initialized = true; // Ensure initialization happens only once
                }

                // Get the current motor position
                int currentPosition = motor.getCurrentPosition();

                // Calculate the error (difference between current and target positions)
                int error = Math.abs(targetPosition - currentPosition);

                // Add telemetry data for debugging
//                telemetry.addData("Current Position", slides.getCurrentPosition());
//                telemetry.addData("Error Position", error);
//                telemetry.update();
                // Check if the motor has reached the target position within the tolerance
                if (error > tolerance) {
                    // Keep running
                    return true;
                } else {
                    // Stop the motor and mark the action as complete
                    motor.setPower(0); // Ensure motor stops
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Prevent unintended behavior
                    return false; // Action is complete
                }
            }
        };
    }
    private Action createStayingMotorAction(DcMotor motor, int targetPosition, double power, int tolerance) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Use brake mode
                    motor.setTargetPosition(targetPosition);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(power);
                    initialized = true; // Ensure initialization happens only once
                }

                int currentPosition = motor.getCurrentPosition();
                int error = Math.abs(targetPosition - currentPosition);

                telemetry.addData("Motor Position", currentPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.addData("Error", error);
                telemetry.update();

                if (error > tolerance) {
                    return true; // Motor still moving
                } else {
                    motor.setPower(0.01); // Stop motor once within tolerance
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset to prevent jitter
                    return false; // Action complete
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




//    Function<Pose2d,Pose2d> mirrorPose = pose-> new Pose2d( -pose.position.x, -pose.position.y, pose.heading.toDouble()-Math.PI);
//    Function<Vector2d,Vector2d> mirrorVector = vector-> new Vector2d( -vector.x, -vector.y);
//
//    /**
//     * Combining multiple Utility methods into one action.
//     */
//    public class ConcurrentAction implements Action {
//        private final List<Action> actions;
//
//        public ConcurrentAction(List<Action> actions) {
//            this.actions = actions;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            boolean anyRunning = false;
//            for (Action action : actions) {
//                if (action.run(packet)) {
//                    anyRunning = true; // At least one action is still running
//                }
//            }
//            return anyRunning; // Continue running until all actions are complete
//        }
//    }

}