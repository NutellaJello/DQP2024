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
import java.util.Collections;
import java.util.List;

@Config
@Autonomous(name = "4 spec")
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
                            intakeRotation.setPosition(0.75);
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
                            intakeSlides.setPosition(0.69);
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
                            outtakeClaw.setPosition(0.31);
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
                            outtakeRotation.setPosition(0.381); //0.383
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
        claw.setPosition(0.4);
        //Hold Spece
        rotation2.setPosition(0.66);
        outtakeClaw.setPosition(0.347);
        // Define custom velocity and acceleration constraints


        int slowVelocity = 60;
        // preload
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(6,-60,Math.toRadians(90)))
                .strafeTo(new Vector2d(-7,-28));
        Action preload=tab1.build();

        // 1st sample
        TrajectoryActionBuilder spec1 = drive.actionBuilder(new Pose2d(-7, -28, Math.toRadians(90)))
                .lineToY(-30)
                .splineToLinearHeading(new Pose2d(28, -44, Math.toRadians(54)), Math.toRadians(90));
        Action intake1 = spec1.build();

        TrajectoryActionBuilder spec1d = drive.actionBuilder(new Pose2d(28, -44, Math.toRadians(54)))
                .turnTo(Math.toRadians(-45));
        Action drop1 = spec1d.build();

        // 2nd sample
        TrajectoryActionBuilder spec2 = drive.actionBuilder(new Pose2d(28, -44, Math.toRadians(-45)))
                .turnTo(Math.toRadians(44));
        Action intake2 = spec2.build();

        TrajectoryActionBuilder spec2d = drive.actionBuilder(new Pose2d(28, -44, Math.toRadians(44)))
                .turnTo(Math.toRadians(-45));
        Action drop2 = spec2d.build();

        // 3rd sample
        TrajectoryActionBuilder spec3 = drive.actionBuilder(new Pose2d(28, -44, Math.toRadians(-45)))
                .splineToLinearHeading(new Pose2d(40, -44, Math.toRadians(41)), Math.toRadians(90));
        Action intake3 = spec3.build();

        TrajectoryActionBuilder spec3d = drive.actionBuilder(new Pose2d(40, -44, Math.toRadians(41)))
                .turnTo(Math.toRadians(-55));
        Action drop3 = spec3d.build();

        // 1st spec
        TrajectoryActionBuilder spec1i = drive.actionBuilder(new Pose2d(40, -43, Math.toRadians(-55)))
                .strafeToLinearHeading(new Vector2d(31,-61.2), Math.toRadians(91));
        Action wall1 = spec1i.build();

        TrajectoryActionBuilder toBar1 = drive.actionBuilder(new Pose2d(31, -61.2, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(3,-28), Math.toRadians(91));
        Action bar1 = toBar1.build();

        TrajectoryActionBuilder shift1 = drive.actionBuilder(new Pose2d(3, -28, Math.toRadians(90)))
                .strafeTo(new Vector2d(3,-31));
        Action back1 = shift1.build();

        // 2nd spec
        TrajectoryActionBuilder spec2i = drive.actionBuilder(new Pose2d(3, -31, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31,-63.5),Math.toRadians(88));
        Action wall2 = spec2i.build();

        TrajectoryActionBuilder toBar2 = drive.actionBuilder(new Pose2d(31, -63.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-0.5,-28), Math.toRadians(88));
        Action bar2 = toBar2.build();

        TrajectoryActionBuilder shift2 = drive.actionBuilder(new Pose2d(-0.5, -28, Math.toRadians(90)))
                .strafeTo(new Vector2d(-0.5,-31));
        Action back2 = shift2.build();

        // 3rd spec
        TrajectoryActionBuilder spec3i = drive.actionBuilder(new Pose2d(-0.5, -31, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31,-63.5),Math.toRadians(87));
        Action wall3 = spec3i.build();

        TrajectoryActionBuilder toBar3 = drive.actionBuilder(new Pose2d(31, -63.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-3,-28.5), Math.toRadians(84));
        Action bar3 = toBar3.build();

        TrajectoryActionBuilder shift3 = drive.actionBuilder(new Pose2d(-3, -28.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-3,-32),Math.toRadians(90));
        Action back3 = shift3.build();

        // park
        TrajectoryActionBuilder goPark = drive.actionBuilder(new Pose2d(-3, -32, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31,-64),Math.toRadians(90));
        Action park = goPark.build();

        // 4th spec?
        TrajectoryActionBuilder spec4i = drive.actionBuilder(new Pose2d(-2, -32, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(31,-64),Math.toRadians(84));
        Action wall4 = spec4i.build();

        TrajectoryActionBuilder toBar4 = drive.actionBuilder(new Pose2d(31, -64, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-4,-27), Math.toRadians(90));
        Action bar4 = toBar4.build();

        TrajectoryActionBuilder shift4 = drive.actionBuilder(new Pose2d(-4, -27, Math.toRadians(90)))
                .strafeTo(new Vector2d(-4,-31));
        Action back4 = shift4.build();



//

        // Use utility methods to create actions
        Action slidesDown=createMotorAction(slides,-120,1,1);
        Action slidesSpecUp=createMotorAction(slides,-660,1,3);
        Action slidesPartDown = createMotorAction(slides, -290, 1,10);

        Action slidesPartUp1 = createMotorAction(slides,-172,1,2);
        Action slidesPartUp2 = createMotorAction(slides,-172,1,2);
        Action slidesPartUp3 = createMotorAction(slides,-172,1,2);
        Action slidesPartUp4 = createMotorAction(slides,-172,1,2);

        Action slidesHang1 =createMotorAction(slides,-330,1,20);
        Action slidesHang2 =createMotorAction(slides,-330,1,20);
        Action slidesHang3 =createMotorAction(slides,-330,1,20);
        Action slidesHang4 =createMotorAction(slides,-330,1,20);

        Action slidesDown1 = createMotorAction(slides,-2 , 1,10);
        Action slidesDown2 = createMotorAction(slides,-2 , 1,10);
        Action slidesDown3 = createMotorAction(slides,-2 , 1,10);
        Action slidesDown4 = createMotorAction(slides,-2 , 1,7);

        Action slidesDownFinal = createMotorAction(slides, 5, 1, 0);




        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(

                        // preload
                        new ParallelAction(preload,slidesSpecUp),
                        slidesPartDown,

                        // 1st sample
                        new ParallelAction(outtakeClaw.openClaw(), intake1,slidesDown, new SequentialAction(
                                new SleepAction(1),
                                new ParallelAction(intakeSlides.moveToPosition2(0.655),pivot.setPosition2(0.72)),
                                intakeRotation.intakeRotDown())),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeRotation.intakeRotPartialUp(),drop1,new SequentialAction(new SleepAction(0.5),intakeClaw.openClaw())),

                        // 2nd sample
                        new ParallelAction(new SequentialAction(new SleepAction(0.32),intakeRotation.intakeRotDown()),intake2, intakeSlides.moveToPosition2(0.77)),
                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeRotation.intakeRotPartialUp(),drop2,new SequentialAction(new SleepAction(0.5),intakeClaw.openClaw())),

                        // 3rd sample
                        new ParallelAction(intake3,  intakeSlides.moveToPosition2(0.66),new SequentialAction(new SleepAction(0.6),
                                new ParallelAction(intakeSlides.moveToPosition2(0.74),intakeRotation.intakeRotDown()))),

                        intakeClaw.closeClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(intakeSlides.moveToPosition2(0.55), intakeRotation.intakeRotPartialUp(),
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        new ParallelAction(drop3,
                                                new SequentialAction(new SleepAction(0.55), intakeClaw.openClaw())))),

                        // 1st spec had intakeRotation.intakeRotUp() in parallelaction
                        new ParallelAction(wall1, outtakeRotation.outtakeRotWall(), outtakeClaw.openClaw(),intakeRotation.intakeRotUp(),intakeSlides.moveToPosition2(0.56),pivot.resetPivot(),
                                new SequentialAction(new SleepAction(0.9),
                                        slidesDown1,
                                        new SleepAction(0.1),
                                        outtakeClaw.closeClaw(),
                                        new SleepAction(0.1))),
                        slidesPartUp1,
                        new ParallelAction(bar1, outtakeRotation.outtakeRotSpec()),
                        new ParallelAction(slidesHang1, new SequentialAction(new SleepAction(0.35),back1, new SleepAction(0.1),outtakeClaw.openClaw()
                        )),

                        // 2nd spec
                        new ParallelAction(wall2, outtakeRotation.outtakeRotWall(),
                                new SequentialAction(new SleepAction(0.7),
                                        slidesDown2,
                                        new SleepAction(0.1),
                                        outtakeClaw.closeClaw(),
                                        new SleepAction(0.1))),
                        slidesPartUp2,
                        new ParallelAction(bar2, outtakeRotation.outtakeRotSpec()),
                        new ParallelAction(slidesHang2, new SequentialAction(new SleepAction(0.35),back2, new SleepAction(0.1),outtakeClaw.openClaw())),

                        // 3rd spec
                        new ParallelAction(wall3, outtakeRotation.outtakeRotWall(),
                                new SequentialAction(new SleepAction(0.7),
                                        slidesDown3,
                                        new SleepAction(0.1),
                                        outtakeClaw.closeClaw(),
                                        new SleepAction(0.1))
                        ),
                        slidesPartUp3,
                        new ParallelAction(bar3, outtakeRotation.outtakeRotSpec()),
                        new ParallelAction(slidesHang3, new SequentialAction(new SleepAction(0.35),back3, new SleepAction(0.1),outtakeClaw.openClaw(),intakeRotation.intakeRotUp())),

                        // park
                        new ParallelAction(park, slidesDownFinal, intakeSlides.retractPosition(), pivot.resetPivot())






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