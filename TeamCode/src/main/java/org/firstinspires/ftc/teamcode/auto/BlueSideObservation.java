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

@Config
@Autonomous(name = "Observation Auto")
public class BlueSideObservation extends LinearOpMode {
    // Declare motors and servos
    private DcMotor slides;
    private Servo slides2, rotation,claw, claw2, rotation2, swing;

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
                            intakeRotation.setPosition(0.78);
                            return false;
                        }}, new SleepAction(0.3)
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
                            intakeSlides.setPosition(0.668);
                            return false;
                        }}, new SleepAction(0.5)
            );
        }
        public Action moveToThirdSample() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.71);
                            return false;
                        }}, new SleepAction(0.2)
            );
        }

        public Action retractPosition() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.5);
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
        public Action outtakeRotSpec() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.371);
                            return false;
                        }}, new SleepAction(0.3)
            );
        }
        public Action outtakeRotTransfer() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.7821);
                            return false;
                        }}, new SleepAction(0.3)
            );
        }
        public Action outtakeRotWall() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            outtakeRotation.setPosition(0.98);
                            return false;
                        }}, new SleepAction(0.3)
            );
        }

        public void setPosition(double value){
            outtakeRotation.setPosition(value);
        }
    }
    public class intakePivot{
        private Servo pivot;
        public intakePivot(HardwareMap hardwareMap) {
            pivot = hardwareMap.get(Servo.class, "pivot");
        }
        public Action resetPivot(){
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            pivot.setPosition(0.53);
                            return false;
                        }}, new SleepAction(0.1)
            );
        }
        public void setPosition(double value){pivot.setPosition(value);
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
        intakePivot pivot = new intakePivot(hardwareMap);

        slides = hardwareMap.get(DcMotor.class, "slides");
        slides2 = hardwareMap.get(Servo.class, "slides2");
        rotation = hardwareMap.get(Servo.class, "rotation");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        rotation2 = hardwareMap.get(Servo.class, "rotation2");
        swing = hardwareMap.get(Servo.class, "swing"); // control hub port 5


        // Configure motor and servo settings
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        swing.setPosition(0.9);
        pivot.setPosition(0.53);
        //Ensure intake does not move around
        intakeSlides.setPosition(0.51);
        claw.setPosition(0.45);
        //Hold Spece
        rotation2.setPosition(0.66);
        outtakeClaw.setPosition(0.347);
        // Define custom velocity and acceleration constraints


        int velocity = 100;
        int slowvelocity = 60;
        // Autonomous Actions
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(6,-60,Math.toRadians(90)))
                .strafeTo(new Vector2d(-5,-28), new TranslationalVelConstraint(velocity));
        Action movement1=tab1.build();

// actions for tranfering and strafing
        TrajectoryActionBuilder spec1 = drive.actionBuilder(new Pose2d(-5, -28, Math.toRadians(90)))
                .lineToY(-30,new TranslationalVelConstraint(velocity))
                .splineToConstantHeading(new Vector2d(42.75,-46), Math.toRadians(90), new TranslationalVelConstraint(velocity));
        Action firstSample = spec1.build();

        TrajectoryActionBuilder spec2 = drive.actionBuilder(new Pose2d(42.75, -46, Math.toRadians(90)))
                .strafeTo(new Vector2d(50,-46), new TranslationalVelConstraint(velocity));
        Action secondSample = spec2.build();

        TrajectoryActionBuilder spec3 = drive.actionBuilder(new Pose2d(50, -46, Math.toRadians(90)))
                .strafeTo(new Vector2d(46,-46), new TranslationalVelConstraint(velocity))
                .turnTo(Math.toRadians(53));
        Action thirdSample = spec3.build();

        TrajectoryActionBuilder spec3p2 = drive.actionBuilder(new Pose2d(46, -46, Math.toRadians(57)))
                .turnTo(Math.toRadians(90));
        Action thirdSampleTransfer = spec3p2.build();

        TrajectoryActionBuilder test = drive.actionBuilder(new Pose2d(46, -46, Math.toRadians(90)))
                .strafeTo(new Vector2d(30,-38), new TranslationalVelConstraint(velocity))
                .strafeTo(new Vector2d(30,-61), new TranslationalVelConstraint(velocity));
        Action pickSpecimenWall = test.build();

        TrajectoryActionBuilder toBar = drive.actionBuilder(new Pose2d(30, -61, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-10,-32.7), Math.toRadians(90))
                .lineToY(-27.4,
                        new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(50), new AngularVelConstraint(Math.toRadians(90)))) ,new ProfileAccelConstraint(-10,10));
        Action moveToBar = toBar.build();

        TrajectoryActionBuilder secondSpec = drive.actionBuilder(new Pose2d(-10, -27.4, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(31.5,-55),Math.toRadians(90))
                .strafeTo(new Vector2d(31.5,-62));
        Action moveTo2ndSpecimen = secondSpec.build();

        TrajectoryActionBuilder toObs2nd = drive.actionBuilder(new Pose2d(31.5, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-8,-32.3), Math.toRadians(90))
                .lineToY(-27,
                        new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(50), new AngularVelConstraint(Math.toRadians(90)))) ,new ProfileAccelConstraint(-10,10));
        Action moveTo2ndObservation = toObs2nd.build();

        TrajectoryActionBuilder park= drive.actionBuilder(new Pose2d(-8, -27, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(50,-60), Math.toRadians(90), new TranslationalVelConstraint(velocity));
        Action goPark = park.build();



        // Use utility methods to create actions
        Action slidesSpecUp=createMotorAction(slides,-660,1);
        Action slidesPartDown = createMotorAction(slides, -290, 1); // Slides partially down
        Action slidesDown = createMotorAction(slides,-5 , 1);       // Slides down

        Action slidesTransfer = createMotorAction(slides,-105 , 1);

        Action slidesPickSpec = createMotorAction(slides,-200,1);

        Action slidesPartUp = createMotorAction(slides,-55,1);
        Action slidesHang =createMotorAction(slides,-628,1,40);
        //Action slidesHang =createMotorActionUsingEncoder(slides,-288,1,3);
        //Action slidesHang =createMotorAction(slides,-248,1);

        Action slidesDown2 = createMotorAction(slides,-5 , 1);

        Action slidesPick2ndSpec = createMotorAction(slides,-240,1, 40);
        Action slidesPartUp2 =createMotorAction(slides,-55,1);

        Action slidesSecondHang =createMotorAction(slides,-688,1, 40);

        Action slidesDown3 = createMotorAction(slides,0 , 1);


        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(movement1,slidesSpecUp),
                        slidesPartDown,
                        outtakeClaw.openClaw(),
                        new SleepAction (0.1),
                        //first sample
                        new ParallelAction(firstSample,slidesTransfer, outtakeRotation.outtakeRotTransfer()),
                        new ParallelAction(intakeSlides.moveToPosition(), intakeRotation.intakeRotDown()),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        intakeClaw.openClaw(),
                        outtakeRotation.outtakeRotWall(),
                        new SleepAction(0.05),
                        outtakeClaw.openClaw(),
                        //second sample
                        new ParallelAction(secondSample, outtakeRotation.outtakeRotTransfer()),
                        new ParallelAction(intakeSlides.moveToPosition(), intakeRotation.intakeRotDown()),
                        new SleepAction (0.1),
                        intakeClaw.closeClaw(),
                        new ParallelAction(intakeRotation.intakeRotUp(), intakeSlides.retractPosition()),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        intakeClaw.openClaw(),
                        outtakeRotation.outtakeRotWall(),
                        new SleepAction(0.05),
                        outtakeClaw.openClaw(),
                        //third sample
                        new ParallelAction( outtakeRotation.outtakeRotTransfer(),thirdSample), // sus
                        new ParallelAction(intakeSlides.moveToThirdSample(), intakeRotation.intakeRotDown()),
                        new SleepAction(0.1),
                        intakeClaw.closeClaw(),
                        new ParallelAction(intakeSlides.retractPosition(), intakeRotation.intakeRotUp()),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        intakeClaw.openClaw(),
                        new ParallelAction(thirdSampleTransfer, outtakeRotation.outtakeRotWall()),
                        new SleepAction(0.01),
                        outtakeClaw.openClaw(),
                        // pick up first Specimen
                        new ParallelAction(pickSpecimenWall, slidesDown),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.2),
                        slidesPickSpec,
                        new ParallelAction(moveToBar, new SequentialAction(outtakeRotation.outtakeRotSpec(),slidesPartUp)),
                        new SleepAction(0.2),
                        slidesHang,
                        new SleepAction(0.2),
                        outtakeClaw.openClaw(),

                        // pick up second Specimen
                        new ParallelAction(moveTo2ndSpecimen, new SequentialAction(outtakeRotation.outtakeRotWall(),slidesDown2)),
                        outtakeClaw.closeClaw(),
                        new SleepAction(0.2),
                        slidesPick2ndSpec,
                        new ParallelAction(moveTo2ndObservation, new SequentialAction(intakeRotation.intakeRotPartialUp(),outtakeRotation.outtakeRotSpec(),slidesPartUp2)),
                        new SleepAction(0.2),
                        slidesSecondHang,
                        new SleepAction(0.2),
                        outtakeClaw.openClaw(),
                        new ParallelAction(goPark,new SequentialAction(intakeRotation.intakeRotUp(),slidesDown3))



                )
        );
    }
//
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
    private Action createMotorAction(DcMotor motor, int targetPosition, double power, int tolerance) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Initialization: Set motor target and mode
                if (!initialized) {
                    motor.setPower(power);
                    motor.setTargetPosition(targetPosition);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    private Action createMotorActionUsingEncoder(DcMotor motor, int targetPosition, double power, int tolerance) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(-power);
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    initialized = true;
                }

                int currentPosition = motor.getCurrentPosition();
                int error = Math.abs(targetPosition - currentPosition);

                // Add telemetry for debugging
                telemetry.addData("Current Position", currentPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.addData("Error Position", error);
                telemetry.update();

                // Check if within tolerance
                if (error <= tolerance) {
                    motor.setPower(0); // Stop motor
                    return false; // Action is complete
                } else {
                    // Adjust power dynamically to maintain torque
                    double adjustedPower = Math.min(Math.abs(power), 1.0); // Clamp power to max 1.0
                    //motor.setPower(error > 0 ? adjustedPower : -adjustedPower); // Adjust direction
                    motor.setPower(-power);
                    return true; // Keep running
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
