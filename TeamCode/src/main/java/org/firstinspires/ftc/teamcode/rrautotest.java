package org.firstinspires.ftc.teamcode;
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

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class rrautotest extends LinearOpMode {
//    public class Lift {
//        private DcMotorEx lift;
//
//        public Lift(HardwareMap hardwareMap) {
//            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
//            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//
//        public class LiftUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    lift.setPower(0.8);
//                    initialized = true;
//                }
//
//                double pos = lift.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos < 3000.0) {
//                    return true;
//                } else {
//                    lift.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action liftUp() {
//            return new LiftUp();
//        }
//
//        public class LiftDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    lift.setPower(-0.8);
//                    initialized = true;
//                }
//
//                double pos = lift.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos > 100.0) {
//                    return true;
//                } else {
//                    lift.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action liftDown(){
//            return new LiftDown();
//        }
//    }
//
//    public class Claw {
//        private Servo claw;
//
//        public Claw(HardwareMap hardwareMap) {
//            claw = hardwareMap.get(Servo.class, "claw");
//        }
//
//        public class CloseClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(0.55);
//                return false;
//            }
//        }
//        public Action closeClaw() {
//            return new CloseClaw();
//        }
//
//        public class OpenClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(1.0);
//                return false;
//            }
//        }
//        public Action openClaw() {
//            return new OpenClaw();
//        }
//    }

    @Override
    public void runOpMode() {
        //Pose2d initialPose = new Pose2d(-16, -60, Math.toRadians(90));
        Pose2d initialPose = new Pose2d(16, -10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // Claw claw = new Claw(hardwareMap);
        // Lift lift = new Lift(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        HSVDetection detectionRed = new HSVDetection(webcam, telemetry);
        webcam.setPipeline(detectionRed);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                // Set MJPEG format to increase FPS
                telemetry.addData("Camera", "Opened successfully");
                telemetry.update();
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open with error code: " + errorCode);
                telemetry.update();
            }

        });


        waitForStart();

        if (isStopRequested()) return;


        // vision here that outputs position
        int visionOutputPosition = 1;
        int cameraCenterX = 320;
        while(opModeIsActive()) {
            int detectedCenterX = detectionRed.getCenterX();  // Get detected object's X position





//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))if (detectedCenterX < cameraCenterX - 10) {
//                // Detected object is to the left, adjust robot to the left
//                drive.strafeLeft(0.2);  // Adjust the speed and method as per your drive system
//            } else if (detectedCenterX > cameraCenterX + 10) {
//                // Detected object is to the right, adjust robot to the right
//                drive.strafeRight(0.2);
//            } else {
//                // Object is centered, stop adjustments
//                drive.stop();
//            }
//                .lineToX(47.5)
//                .waitSeconds(3);
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//                .lineToY(37)
//                .setTangent(Math.toRadians(0))
//                .lineToX(18)
//                .waitSeconds(3)
//                .setTangent(Math.toRadians(0))
//                .lineToXSplineHeading(46, Math.toRadians(180))
//                .waitSeconds(3);
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3);
            TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose);
            if (detectedCenterX != -1) {  // Object is detected
                if (detectedCenterX < cameraCenterX - 10) {
                    // Object is to the left, turn left slightly
                    tab3.turn(Math.toRadians(5));  // Adjust the angle based on your requirements
                } else if (detectedCenterX > cameraCenterX + 10) {
                    // Object is to the right, turn right slightly
                    tab3.turn(Math.toRadians(-5));  // Adjust the angle as needed
                } else {
                    // Object is centered, continue straight or no adjustment
                    tab3.waitSeconds(1);  // Example movement to go forward
                }
            } else {
                telemetry.addData("No object detected", true);
            }

            tab3.turnTo(Math.toRadians(90));
  /* created path for cycling
                .splineTo(new Vector2d(-8, -40), Math.toRadians(90))

                .waitSeconds(1)

                .strafeTo(new Vector2d(-50,-40))


                .waitSeconds(1)

                .splineToLinearHeading(new Pose2d(-58,-53,Math.toRadians(45)),0)

                .waitSeconds(1)

                .turnTo(Math.toRadians(90))

                .waitSeconds(2)

                .turnTo(Math.toRadians(45))

                .waitSeconds(2)

                .turnTo(Math.toRadians(120))

                .waitSeconds(2)

                .turnTo(Math.toRadians(45));
*/


            Action trajectoryActionCloseOut = tab3.fresh()
                    // .strafeTo(new Vector2d(48, 12))
                    .build();

            // actions that need to happen on init; for instance, a claw tightening.
            //  Actions.runBlocking(claw.closeClaw());


            while (!isStopRequested() && !opModeIsActive()) {
                int position = visionOutputPosition;
                telemetry.addData("Position during Init", position);
                telemetry.update();
            }

            int startPosition = 3;
            telemetry.addData("Starting Position", startPosition);
            telemetry.update();
            waitForStart();

            if (isStopRequested()) return;

            Action trajectoryActionChosen;
            if (startPosition == 1) {
                trajectoryActionChosen = tab3.build();
            } else if (startPosition == 2) {
                trajectoryActionChosen = tab3.build();
            } else {
                trajectoryActionChosen = tab3.build();
            }

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionChosen,
                            //   lift.liftUp(),
                            // claw.openClaw(),
                            // lift.liftDown(),
                            trajectoryActionCloseOut
                    )
            );

        }
    }
}