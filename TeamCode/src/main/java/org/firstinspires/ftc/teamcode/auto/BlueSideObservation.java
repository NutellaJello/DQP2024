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
                            intakeSlides.setPosition(0.66);
                            return false;
                        }}, new SleepAction(0.5)
            );
        }

        public Action moveToPosition2(double value) {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(value);
                            return false;
                        }}, new SleepAction(0.5)
            );
        }

        public Action moveToThirdSample() {
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            intakeSlides.setPosition(0.7);
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
                            outtakeRotation.setPosition(0.39);
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
        }public Action setPosition2(double value){
            return new SequentialAction(
                    new Action(){
                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                            pivot.setPosition(value);
                            return false;
                        }}
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
        int slowVelocity = 60;
        // preload
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(6,-60,Math.toRadians(90)))
                .strafeTo(new Vector2d(-5.8,-28), new TranslationalVelConstraint(velocity));
        Action preload=tab1.build();

// 1st sample
        TrajectoryActionBuilder spec1 = drive.actionBuilder(new Pose2d(-5.8, -28, Math.toRadians(90)))
                .lineToY(-30,new TranslationalVelConstraint(velocity))
                .splineToLinearHeading(new Pose2d(27, -43, Math.toRadians(46)), Math.toRadians(90), new TranslationalVelConstraint(velocity));
        Action intake1 = spec1.build();

        TrajectoryActionBuilder spec1d = drive.actionBuilder(new Pose2d(30, -43, Math.toRadians(46)))
                .turnTo(Math.toRadians(-55));
        Action drop1 = spec1d.build();

        // 2nd sample
        TrajectoryActionBuilder spec2 = drive.actionBuilder(new Pose2d(30, -43, Math.toRadians(-55)))
                .turnTo(Math.toRadians(40));
        Action intake2 = spec2.build();

        TrajectoryActionBuilder spec2d = drive.actionBuilder(new Pose2d(30, -43, Math.toRadians(40)))
                .turnTo(Math.toRadians(-55));
        Action drop2 = spec2d.build();

        // 3rd sample
        TrajectoryActionBuilder spec3 = drive.actionBuilder(new Pose2d(30, -43, Math.toRadians(-55)))
                .splineToLinearHeading(new Pose2d(46, -46, Math.toRadians(50)), Math.toRadians(90), new TranslationalVelConstraint(velocity));
        Action intake3 = spec3.build();

        TrajectoryActionBuilder spec3t = drive.actionBuilder(new Pose2d(46, -46, Math.toRadians(50)))
                .turnTo(Math.toRadians(90));
        Action turn3 = spec3t.build();

        TrajectoryActionBuilder test = drive.actionBuilder(new Pose2d(46, -46, Math.toRadians(90)))
                .strafeTo(new Vector2d(30,-38), new TranslationalVelConstraint(velocity))
                .strafeTo(new Vector2d(30,-61), new TranslationalVelConstraint(velocity));
        Action pickSpecimenWall = test.build();

        TrajectoryActionBuilder toBar = drive.actionBuilder(new Pose2d(30, -61, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-10,-33.2), Math.toRadians(90))
                .lineToY(-29,
                        new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(50), new AngularVelConstraint(Math.toRadians(90)))) ,new ProfileAccelConstraint(-10,10));
        Action moveToBar = toBar.build();

        TrajectoryActionBuilder shift1 = drive.actionBuilder(new Pose2d(-10, -29, Math.toRadians(90)))
                .strafeTo(new Vector2d(-10,-32));
        Action shiftBack = shift1.build();

        TrajectoryActionBuilder secondSpec = drive.actionBuilder(new Pose2d(-10, -32, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(31.5,-55),Math.toRadians(90))
                .strafeTo(new Vector2d(31.5,-62));
        Action moveTo2ndSpecimen = secondSpec.build();

        TrajectoryActionBuilder toObs2nd = drive.actionBuilder(new Pose2d(31.5, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-8,-33.3), Math.toRadians(90))
                .lineToY(-29,
                        new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(50), new AngularVelConstraint(Math.toRadians(90)))) ,new ProfileAccelConstraint(-10,10));
        Action moveTo2ndObservation = toObs2nd.build();

        TrajectoryActionBuilder shift2 = drive.actionBuilder(new Pose2d(-8, -29, Math.toRadians(90)))
                .strafeTo(new Vector2d(-8,-32));
        Action shiftBack2 = shift2.build();


        TrajectoryActionBuilder park= drive.actionBuilder(new Pose2d(-8, -32, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(50,-60), Math.toRadians(90), new TranslationalVelConstraint(velocity));
        Action goPark = park.build();

        TrajectoryActionBuilder park2= drive.actionBuilder(new Pose2d(46, -46, Math.toRadians(90)))
                .lineToY(-30);

        Action gotempPark = park2.build();

        TrajectoryActionBuilder park3= drive.actionBuilder(new Pose2d(46, -30, Math.toRadians(90)))
                .lineToY(-55);

        Action gotempPark2 = park3.build();


//

        // Use utility methods to create actions

        Action slidesDown = createMotorAction(slides,-130 , 0.9,1);
        Action slidesPartUp = createMotorAction(slides,-1000,0.8);
        Action slidesPartDown = createMotorAction(slides, -400, 0.3);
//        Action slidesSpecUp=createMotorAction(slides,-660,1);
//        Action slidesPartDown = createMotorAction(slides, -290, 1); // Slides partially down
//        Action slidesDown = createMotorAction(slides,-5 , 1);       // Slides down
//
//        Action slidesTransfer = createMotorAction(slides,-130 , 1);
//
//        Action slidesPickSpec = createMotorAction(slides,-200,1);
//
//        Action slidesPartUp = createMotorAction(slides,-55,1);
//        Action slidesHang =createMotorAction(slides,-260,1,15);
//        //Action slidesHang =createMotorActionUsingEncoder(slides,-288,1,3);
////        Action slidesHang =createMotorAction(slides,-288,1);
//
//        Action slidesDown2 = createMotorAction(slides,-5 , 1);
//
//        Action slidesPick2ndSpec = createMotorAction(slides,-200,1, 5);
//        Action slidesPartUp2 =createMotorAction(slides,-55,1);
//
//        Action slidesSecondHang =createMotorAction(slides,-260,1, 40);
//
//        Action slidesDown3 = createMotorAction(slides,0 , 1);


        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                    preload,

                    new ParallelAction(intake1,slidesDown),
                    new ParallelAction(intakeSlides.moveToPosition2(0.63),pivot.setPosition2(0.7)),
                    intakeRotation.intakeRotDown(),
                    intakeClaw.closeClaw(),
                    new SleepAction(0.1),
                    intakeRotation.intakeRotPartialUp(),
                    drop1,
                    intakeClaw.openClaw(),

                    intake2,
                    intakeSlides.moveToPosition2(0.7),
                    intakeRotation.intakeRotDown(),
                    intakeClaw.closeClaw(),
                    new SleepAction(0.1),
                    intakeRotation.intakeRotPartialUp(),
                    drop2,
                    intakeClaw.openClaw(),
                    new ParallelAction(intakeSlides.retractPosition(), intakeRotation.intakeRotUp(), pivot.resetPivot()),


                    new ParallelAction(outtakeRotation.outtakeRotTransfer(), intake3), // sus
                    new ParallelAction(intakeSlides.moveToThirdSample(), intakeRotation.intakeRotDown()),
                    intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                    new ParallelAction(intakeSlides.retractPosition(), intakeRotation.intakeRotUp()),
                    new SleepAction (0.1),
                    outtakeClaw.closeClaw(),
                    new SleepAction(0.2),
                    intakeClaw.openClaw(),
                    new ParallelAction(turn3, outtakeRotation.outtakeRotWall()),
                    outtakeClaw.openClaw()
//                    new ParallelAction(intakeSlides.moveToPosition(), pivot.setPosition2(0.7)),
//                        intakeRotation.intakeRotDown(),
//                        intakeClaw.closeClaw(),
//                        new SleepAction(0.1),
//                        intakeRotation.intakeRotPartialUp(),
//                        firstTurn,
//                        intakeClaw.openClaw()
////                        secondSample,
////                        intakeRotation.intakeRotDown(),
////                        intakeClaw.closeClaw(),
////                        new SleepAction(0.1),
////                        intakeRotation.intakeRotPartialUp(),
////                        new ParallelAction(secondTurn,intakeSlides.moveToPosition2(0.6)),
////                        new ParallelAction(intakeSlides.moveToPosition(),new SequentialAction(new SleepAction(0.2), intakeClaw.openClaw())),
////                        new ParallelAction(thirdSample, intakeSlides.moveToPosition2(0.55)),
////

                )
        );
    }
////
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
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
//               telemetry.addData("Current Position", slides.getCurrentPosition());
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
//                telemetry.addData("Current Position", currentPosition);
//                telemetry.addData("Target Position", targetPosition);
//                telemetry.addData("Error Position", error);
//                telemetry.update();

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
