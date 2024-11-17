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

    @Override
    public void runOpMode() {
        // Initialize drivetrain and mechanisms
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16, -60, Math.toRadians(270)));

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
        slides2.setPosition(0.69);

        //Hold Spece
        rotation2.setPosition(0.173);
        claw2.setPosition(0.25);


        // Autonomous Actions
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-16, -60, Math.toRadians(270)))
                .setReversed(true)
                //.splineTo(new Vector2d(-8, -32), Math.toRadians(90))
                .lineToY(-32)
                .waitSeconds(1);
        Action movement1 = tab1.build();

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-8, -30, Math.toRadians(270)))
                .lineToY(-53)
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(-45,-53))
                .waitSeconds(1);
        Action movement2 = tab2.build();

        // Use utility methods to create actions
        Action slidesUp = createMotorAction(slides, -1100, 0.8);      // Slides up
        Action slidesPartDown = createMotorAction(slides, -400, 0.6); // Slides partially down
        Action slidesDown = createMotorAction(slides,-10 , 0.6);       // Slides down
        Action slides2out = createServoAction(slides2, 0.4); // PLEASE ADJUST!!!
        Action slides2in = createServoAction(slides2, 0.70); // PLEASE ADJUST!!!
        Action openClaw = createServoAction(claw, 0.54);             // Open claw
        Action closeClaw = createServoAction(claw, 0.25);
        Action rotate2Base = createServoAction(rotation2, 0);           // outtake rotate to equilibrium
        Action rotateBase = createServoAction(rotation, 0.4); // CHANGE

        ArrayList<Action> pickSampleActions = new ArrayList<>();
        pickSampleActions.add(slides2out);
        pickSampleActions.add(rotateBase);
        pickSampleActions.add(slidesDown);

        Action pickSample = new ConcurrentAction(pickSampleActions);


        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        movement1,   // Moving to the submersible
                        slidesUp,
                        slidesPartDown,
                        openClaw,
                        rotate2Base,
                        movement2
                        // intake out
                        // rotation to pick sample
                        // open then close the claw
                        // rotate back
                        // retract intake
                        // transfer to outtake claw
                        // spline to position, check MeepMeep
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
     * Utility method to create an action for a servo to move to a target position.
     */
    private Action createServoAction(Servo servo, double position) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(position);
                return false;  // Single execution
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
