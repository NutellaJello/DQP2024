package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Claws.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Claws.OutakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Rotations.IntakeClawRotation;
import org.firstinspires.ftc.teamcode.subsystems.Rotations.IntakePitchArm;
import org.firstinspires.ftc.teamcode.subsystems.Rotations.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.States.SampleManualState;
import org.firstinspires.ftc.teamcode.subsystems.States.SampleSlideDownState;
import org.firstinspires.ftc.teamcode.subsystems.States.SampleSlideUpState;
import org.firstinspires.ftc.teamcode.subsystems.States.SampleTransferState;
import org.firstinspires.ftc.teamcode.subsystems.States.SpecimenTransferState;
import org.firstinspires.ftc.teamcode.subsystems.States.StateManager;
import org.firstinspires.ftc.teamcode.subsystems.slides.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.OuttakeSlide;

public class IntakeOuttakeSys {
    private IntakeClaw intakeClaw;
    private IntakeSlide intakeSlide;
    private IntakePitchArm intakePitchArm;
    private IntakeClawRotation intakeClawRotation;
    private OuttakeSlide outtakeSlide;
    private OuttakeArm outtakeArm;
    private OutakeClaw outtakeClaw;

    private StateManager stateManager;
    private boolean isOuttakeSlideInNeutral = false;
    private double a2=0; // sample transfer delay variable

    public IntakeOuttakeSys(HardwareMap hardwareMap) {
        intakeClaw = new IntakeClaw(hardwareMap);
        intakeSlide = new IntakeSlide(hardwareMap);
        intakePitchArm = new IntakePitchArm(hardwareMap);
        intakeClawRotation = new IntakeClawRotation(hardwareMap);
        outtakeSlide = new OuttakeSlide(hardwareMap);
        outtakeArm = new OuttakeArm(hardwareMap);
        outtakeClaw = new OutakeClaw(hardwareMap);
        stateManager = new StateManager();
    }

    public void Initialize()
    {
        intakeClaw.Initialize();
        intakePitchArm.Initialize();
        intakeClawRotation.Initialize();
        outtakeSlide.Initalize();
        outtakeArm.Initialize();
        outtakeClaw.Intialize();

        // initialize states
        stateManager.setSampleManualState(SampleManualState.SAMPLE_MANUAL_INIT);
        stateManager.setSampleSlideDownState(SampleSlideDownState.SAMPLE_SLIDE_DOWN_INIT);
        stateManager.setSampleSlideUpState(SampleSlideUpState.SAMPLE_SLIDE_UP_INIT);
        stateManager.setSampleTransferState(SampleTransferState.SAMPLE_TRANSFER_INIT);
        stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_TRANSFER_INIT);
    }

    public void Teleop(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        Teleop(gamepad1, gamepad2, telemetry, false);
    }

    public void Teleop(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean showTelemetry){
        if(!isOuttakeSlideInNeutral) {
            outtakeSlide.MoveToNeutralPos();
            isOuttakeSlideInNeutral = true;
        }

        telemetry.addData("a1", stateManager.getSpecimenTransferState());
        intakeSlide.Teleop(gamepad2, stateManager, intakePitchArm, intakeClaw, telemetry);
        outtakeSlide.TeleopSample(gamepad1, gamepad2, stateManager, outtakeArm, outtakeClaw, telemetry);
        intakePitchArm.Teleop(gamepad2, telemetry);
        outtakeClaw.Teleop(gamepad2, telemetry);
        outtakeArm.Teleop(gamepad2, telemetry);
        intakeClaw.Teleop(gamepad2, telemetry);
        intakeClawRotation.Teleop(gamepad2, telemetry);

        outtakeSlide.TeleopSpecimen(gamepad2, stateManager, outtakeArm, outtakeClaw, telemetry);
        //telemetry.addData("hang", hang.getCurrentPosition());


        // AUTOMATION STUFF
        if (gamepad2.right_stick_button) {
            //transfer()
//            slides.setTargetPosition(slidesnuetral);
//            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slides.setPower(0.8);
            outtakeSlide.goToNeutralPosition();
//            claw2pos=claw2open;
//            rot2pos=rot2down;
//            rotation2.setPosition(rot2pos);
//            claw2.setPosition(claw2pos);
//            clawpos=clawclose;
//            claw.setPosition(clawpos);
            outtakeClaw.OpenClaw();
            outtakeArm.SetPosToDown();
            intakeClaw.CloseClaw();

            //picked up sample on floor
            //a1=1;
            stateManager.setSampleTransferState(SampleTransferState.SAMPLE_TRANSFER_FLOOR_CLAMP);
        }
        if(stateManager.getSampleTransferState()==SampleTransferState.SAMPLE_TRANSFER_FLOOR_CLAMP){
            a2+=0.015;
            if(a2>=0.2) {

                //if( (claw.getPosition()-clawpos)<= 0.0001 ){
                if(intakeClaw.PositionReachedWithTolerance(0.0001)){
                    //a1=2;
                    stateManager.setSampleTransferState(SampleTransferState.SAMPLE_TRANSFER_PICKUP_NEUTRAL_ANGLING);
//                    rotpos=rotin;
//                    rotation.setPosition(rotpos);
                    intakePitchArm.PitchIn();
//                    pivotpos=pivotnuetral;
//                    pivot.setPosition(pivotpos);
                    intakeClawRotation.PivotNeutral();
                    a2=0;
                    //rotating up and pivot nuetral
                }

            }

        }

        //waiting for the rotation to finish
        if(stateManager.getSampleTransferState()==SampleTransferState.SAMPLE_TRANSFER_PICKUP_NEUTRAL_ANGLING){
            a2+=0.015;
            if(a2>=0.3) {
                a2=0;
                //a1=3;
                stateManager.setSampleTransferState(SampleTransferState.SAMPLE_TRANSFER_PICKUP_NEUTRAL_ANGLE_DONE);
            }
        }
/*
        if(stateManager.getSampleTransferState()==SampleTransferState.SAMPLE_TRANSFER_PICKUP_NEUTRAL_ANGLING){
            //if( (rotation.getPosition()-rotpos)<= 0.00001 ){
            if(intakePitchArm.PositionReachedWithTolerance(0.00001)){
                //a1=3;
                stateManager.setSampleTransferState(SampleTransferState.SAMPLE_TRANSFER_PICKUP_NEUTRAL_ANGLE_DONE);
//                slides2pos=slides2in;
//                slides2.setPosition(slides2pos);
                intakeSlide.SlideIn();
                //sending slides in`
            }
        }
*/
        if(stateManager.getSampleTransferState()==SampleTransferState.SAMPLE_TRANSFER_PICKUP_NEUTRAL_ANGLE_DONE){
            //if ((slides2.getPosition() - slides2pos) <= 0.00001) {
            if(intakeSlide.PositionReachedWithTolerance(0.00001)){
                a2 += 0.015;
                if (a2 >= 0.7) {
                    //waiting for slides to arrive
//                    claw2pos = claw2close;
//                    claw2.setPosition(claw2pos);
                    outtakeClaw.CloseClaw();
                    //a1 = 4;
                    stateManager.setSampleTransferState(SampleTransferState.SAMPLE_TRANSFER_OUTTAKE_CLOSE);
                    a2 = 0;
                    //outtake claw closing up
                }
            }

        }

        if(stateManager.getSampleTransferState()==SampleTransferState.SAMPLE_TRANSFER_OUTTAKE_CLOSE){
            a2+=0.015;
            if(a2>=0.13) {
//                clawpos = clawopen;
//                claw.setPosition(clawpos);
                intakeClaw.OpenClaw();
                //a1 = 5;
                stateManager.setSampleTransferState(SampleTransferState.SAMPLE_TRANSFER_INTAKE_OPEN);
                a2=0;
                //intake claw opening
            }
        }

        if(stateManager.getSampleTransferState()==SampleTransferState.SAMPLE_TRANSFER_INTAKE_OPEN){
            a2+=0.015;
            if(a2>=0.3) {
                //waiting for intake claw to open to slide away
//                slides2pos=0.5;
//                slides2.setPosition(slides2pos);
                intakeSlide.setPosition(0.5);
                //a1 = 0;
                stateManager.setSampleTransferState(SampleTransferState.SAMPLE_TRANSFER_INIT);
                a2=0;

            }
        }

    }

    public void HangPreventCollision()
    {
//        rot2pos = 0.85;
//        rotation2.setPosition(rot2pos);
        outtakeArm.setPosition(0.85);
//        clawpos = clawclose;
//        claw.setPosition(clawpos);
        intakeClaw.CloseClaw();
//        slides2pos=slides2in+0.14;
//        slides2.setPosition(slides2pos);
        intakeSlide.SlideInTweak();
    }
}
