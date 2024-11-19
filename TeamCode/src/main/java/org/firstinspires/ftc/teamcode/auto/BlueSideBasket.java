package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeClaw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
    }

    public class intakeRotation {
        private Servo intakeRotation;

        public intakeRotation(HardwareMap hardwareMap) {
            intakeRotation = hardwareMap.get(Servo.class, "rotation");
        }
        public class IntakeRotDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRotation.setPosition(0.25);
                return false;
            }
        }
        public Action intakeRotDown() {
            return new IntakeRotDown();
        }

        public class IntakeRotUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRotation.setPosition(0.97);
                return false;
            }
        }
        public Action intakeRotUp() {
            return new IntakeRotDown();
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

        public class MoveOutIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeSlides.setPosition(0.5726);
                return false; // Single execution, action is complete
            }
        }

        public Action moveToPosition() {
            return new MoveOutIntake();
        }
        public class RetractIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeSlides.setPosition(0.69);
                return false; // Single execution, action is complete
            }
        }

        public Action retractPosition() {
            return new RetractIntake();
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
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClaw.setPosition(0.25);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClaw.setPosition(0.54);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
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
        public class OuttakeRotDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeRotation.setPosition(0.25);
                return false;
            }
        }
        public Action outtakeRotDown() {
            return new OuttakeRotDown();
        }

        public class OuttakeRotUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeRotation.setPosition(0.97);
                return false;
            }
        }
        public Action outtakeRotUp() {
            return new OuttakeRotUp();
        }
        public void setPosition(double value){
            outtakeRotation.setPosition(value);
        }
    }




    @Override
    public void runOpMode() {
        // Initialize drivetrain and mechanisms
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16, -60, Math.toRadians(270)));

        intakeClaw intakeClaw = new intakeClaw(hardwareMap);
        intakeRotation intakeRotation = new intakeRotation(hardwareMap);
        intakeSlides intakeSlides = new intakeSlides(hardwareMap);

        outtakeClaw outtakeClaw = new outtakeClaw(hardwareMap);
        //outtakeRotation outtakeRotation = new outtakeRotation(hardwareMap);

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
        rotation2.setPosition(0.173);
        outtakeClaw.setPosition(0.25);


        // Autonomous Actions
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-16, -60, Math.toRadians(270)))
                .setReversed(true)
                .lineToY(-29)
                .waitSeconds(1);
        Action movement1 = tab1.build();

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-16, -29, Math.toRadians(270)))
                .lineToY(-40)
                .turn(Math.toRadians(179.5))
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(-50,-40))
                .waitSeconds(1);
        Action movement2 = tab2.build();

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-45, -53, Math.toRadians(90)))
                .splineTo(new Vector2d(-60, -53), Math.toRadians(45));
        Action movement3 = tab3.build();

        // Use utility methods to create actions
        Action slidesUp = createMotorAction(slides, -1100, 0.8);      // Slides up
        Action slidesPartDown = createMotorAction(slides, -275, 0.2); // Slides partially down
        Action slidesDown = createMotorAction(slides,-10 , 0.6);       // Slides down

        ArrayList<Action> pickSampleActions = new ArrayList<>();
        pickSampleActions.add(slidesDown);

        Action pickSample = new ConcurrentAction(pickSampleActions);


        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        slidesUp,
                         movement1,   // Moving to the submersible
                        slidesPartDown,
                        outtakeClaw.openClaw(),
                      //  rotate2Base,
                        movement2,
                        intakeSlides.moveToPosition(),
                        intakeRotation.intakeRotDown(),
                        intakeClaw.closeClaw(),
                        intakeRotation.intakeRotUp(),
                        intakeSlides.retractPosition()
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
