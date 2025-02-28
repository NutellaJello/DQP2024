package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

@Config
@Autonomous(name = "1+3 basket auto")
public class BlueSideBasket extends LinearOpMode {

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
                            intakeSlides.setPosition(0.63);
                            return false;
                        }}, new SleepAction(0.4)
            );
        }


        public Action retractPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.505);
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
                            outtakeClaw.setPosition(0.51);
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
                            outtakeClaw.setPosition(0.35);
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
                            outtakeRotation.setPosition(0.761);
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
                        }}, new SleepAction(0.65)
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16, -60, Math.toRadians(270)));


        intakeClaw intakeClaw = new intakeClaw(hardwareMap);
        intakeRotation intakeRotation = new intakeRotation(hardwareMap);
        intakeSlides intakeSlides = new intakeSlides(hardwareMap);
        intakePivot intakePivot = new intakePivot(hardwareMap);

        outtakeClaw outtakeClaw = new outtakeClaw(hardwareMap);
        outtakeRotation outtakeRotation = new outtakeRotation(hardwareMap);


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

        //Ensure intake does not move around
        intakeSlides.setPosition(0.4968);
        claw.setPosition(0.45);
        swing.setPosition(1);
        //Hold Spece
        rotation2.setPosition(1);
        outtakeClaw.setPosition(0.523);

        // Autonomous Actions
       //hang preload
        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-16,-60,Math.toRadians(270)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-15,-33.6), Math.toRadians(270),new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(35), new AngularVelConstraint(Math.toRadians(90)))) ,new ProfileAccelConstraint(-35,35));
        Action movement=tab.build();
        // move to first sample
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-15, -33.6, Math.toRadians(270)))
                .lineToY(-40)
                .turnTo(Math.toRadians(87))
                .setReversed(false)
                .strafeTo(new Vector2d(-42.5,-43));
        Action movement1 = tab1.build();
//      bucket first sample
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-42.5, -43, Math.toRadians(90)))
                .strafeTo(new Vector2d(-61,-47));
        Action movement2 = tab2.build();
        // move to second sample
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-61, -47, Math.toRadians(90)))
                .strafeTo(new Vector2d(-52.3,-42));
        Action movement3 = tab3.build();
        // move to second bucket
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-52.3,-42, Math.toRadians(90)))
                .strafeTo(new Vector2d(-60,-47));
        Action movement5 = tab5.build();
        // third sample
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-60,-47, Math.toRadians(90)))
                .strafeTo(new Vector2d(-64.5,-42));
        Action movement6 = tab6.build();
        // third bucket
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-64.5,-42, Math.toRadians(90)))
                .strafeTo(new Vector2d(-60,-47));
        Action movement7 = tab7.build();
        // park!
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(-60,-47, Math.toRadians(90)))
                .splineTo(new Vector2d(-30,-9), Math.toRadians(0));
        Action movement8 = tab8.build();




////

        // Use utility methods to create actions
              // Slides up for specimen
        Action slidesSpecUp = createMotorAction(slides, -600, 1);
        Action slidesPartiallyDown = createMotorAction(slides, -200, 1,10);

        // first sample
        Action slidesDown = createStayingMotorAction(slides,-240 , 0.9,1);
        Action slidesUp = createStayingMotorAction(slides, -890, 1,5);
              // Slides down
        // second sample
        Action slidesDown2 = createMotorAction(slides,-244 , 0.9,1);
        Action slidesUp2 = createStayingMotorAction(slides, -890, 1,5);
        // third sample
        Action slidesDown3 = createMotorAction(slides,-243 , 0.9,1);
        Action slidesUp3 = createStayingMotorAction(slides, -890, 1,5);
        // ensure slides are all the way down
        Action slidesDown4 = createMotorAction(slides,5 , 1,1);



        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        // go to hang the first specimen
                        new ParallelAction(movement, slidesSpecUp),
                        slidesPartiallyDown,
                        new SleepAction(0.1),
                        outtakeClaw.openClaw(),
                        // moving to pick up first sample
                        movement1,
                        new ParallelAction(intakeSlides.moveToPosition(), outtakeRotation.outtakeRotTransfer()),
                        //new ParallelAction(intakeRotation.intakeRotDown(),slidesDown),
                        intakeRotation.intakeRotDown(),
                        slidesDown,
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                        new SleepAction(0.3),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.4),
                        intakeClaw.openClaw(),
                        //first sample scored
                        new ParallelAction(movement2,
                                slidesUp),
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotTransfer(),
                        // moving to second sample
                        new ParallelAction(movement3,
                                slidesDown2
                        ),
                        intakeSlides.moveToPosition(),
                        new SleepAction(0.3),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.3),
                        intakeClaw.openClaw(),
                        // moving to score 2nd sample
                        new ParallelAction(movement5,
                                slidesUp2
                        ),
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotTransfer(),
                        // moving to 3rd sample
                        new ParallelAction(movement6, slidesDown3),
                        new ParallelAction(intakeSlides.moveToPosition(), intakePivot.resetPivot()),
                        new SleepAction(0.3),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.3),
                        intakeClaw.openClaw(),
                        //move to score 3rd sample
                        new ParallelAction(movement7,
                                slidesUp3
                        ),
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotFinal(),
//                      go park in the zone for easy teleop
                        new ParallelAction(movement8,
                                slidesDown4
                        )


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
                if (!initialized) {// Use brake mode
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Use brake mode
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
                telemetry.addData("Current Position", slides.getCurrentPosition());
                telemetry.addData("Error Position", error);
                telemetry.update();
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

}