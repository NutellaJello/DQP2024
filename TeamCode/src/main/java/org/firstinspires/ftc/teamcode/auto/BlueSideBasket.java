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
@Autonomous(name = "Basket Auto")
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
                            intakeSlides.setPosition(0.61);
                            return false;
                        }}, new SleepAction(0.4)
            );
        }


        public Action retractPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.49);
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

//


    @Override
    public void runOpMode() {
        // Initialize drivetrain and mechanisms
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16, -60, Math.toRadians(270)));


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
        swing = hardwareMap.get(Servo.class, "swing"); // control hub port 5



        // Configure motor and servo settings
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setPosition(0.53);

        //Ensure intake does not move around
        intakeSlides.setPosition(0.4968);
        claw.setPosition(0.45);
        swing.setPosition(1);
        //Hold Spece
        rotation2.setPosition(0.98);
        outtakeClaw.setPosition(0.347);

        // Autonomous Actions

        TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(-16,-60,Math.toRadians(270)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-15,-31), Math.toRadians(270), new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(50), new AngularVelConstraint(Math.toRadians(90)))) ,new ProfileAccelConstraint(-30,30)
       );
        Action movement=tab.build();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-15, -31, Math.toRadians(270)))
                .lineToY(-40)
                .turnTo(Math.toRadians(90))
                .setReversed(false)
                .strafeTo(new Vector2d(-43,-42.4));
        Action movement1 = tab1.build();
//
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-43, -42.4, Math.toRadians(90)))
                .strafeTo(new Vector2d(-58.5,-55.5))
                .turnTo(45);
        Action movement2 = tab2.build();

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-58.5, -55.5, Math.toRadians(45)))
                .splineTo(new Vector2d(-53.5,-42.4), Math.toRadians(90));
        Action movement3 = tab3.build();

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-53.5,-42.4, Math.toRadians(90)))
                .strafeTo(new Vector2d(-58.5,-55.5))
                .turnTo(45);
        Action movement5 = tab5.build();

        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-58.5,-55.5, Math.toRadians(45)))
                .turnTo(Math.toRadians(90))
                .splineTo(new Vector2d(-64,-42.4), Math.toRadians(90));
        Action movement6 = tab6.build();

        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-64,-42.4, Math.toRadians(90)))
                .lineToY(-54);
        Action movement7 = tab7.build();
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(-63,-54, Math.toRadians(90)))
                .splineTo(new Vector2d(-30,0), Math.toRadians(0));
        Action movement8 = tab8.build();






        // Use utility methods to create actions
        Action slidesUp = createStayingMotorAction(slides, -890, 1,1);      // Slides up
        Action slidesSpecUp = createMotorAction(slides, -680, 0.8,1);
        Action slidesDown = createMotorAction(slides,-105 , 0.9,1);       // Slides down
        Action slidesPartiallyDown = createMotorAction(slides, -240, 0.8);
        Action slidesDown2 = createMotorAction(slides,-105 , 0.9,1);
        Action slidesUp2 = createStayingMotorAction(slides, -890, 1,1);
        Action slidesDown3 = createMotorAction(slides,-105 , 0.9,1);
        Action slidesUp3 = createStayingMotorAction(slides, -890, 1,1);
        Action slidesDown4 = createMotorAction(slides,0 , 1,1);



        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(movement, slidesSpecUp),
                        slidesPartiallyDown,
                        outtakeClaw.openClaw(),
                        new ParallelAction(movement1),
                        new ParallelAction(intakeSlides.moveToPosition(), outtakeRotation.outtakeRotTransfer()),
                        new ParallelAction(intakeRotation.intakeRotDown(),slidesDown),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.3),
                        intakeClaw.openClaw(),
                        new ParallelAction(movement2,
                                slidesUp),
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotTransfer(),
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
                        new ParallelAction(movement5,
                                slidesUp2
                        ),
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotTransfer(),
                        new ParallelAction(movement6, slidesDown3),
                        intakeSlides.moveToPosition(),
                        new SleepAction(0.3),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.3),
                        intakeClaw.openClaw(),
                        new ParallelAction(movement7,
                                slidesUp3
                        ),
                        outtakeRotation.outtakeRotBasket(),
                        outtakeClaw.openClaw(),
                        outtakeRotation.outtakeRotFinal(),
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
                if (!initialized) {
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
                    motor.setPower(0); // Ensure the motor stops when done
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
                    motor.setPower(0.1); // Stop motor once within tolerance
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