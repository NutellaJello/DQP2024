package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Claws.OutakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Rotations.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.States.SampleSlideDownState;
import org.firstinspires.ftc.teamcode.subsystems.States.SampleSlideUpState;
import org.firstinspires.ftc.teamcode.subsystems.States.SpecimenTransferState;
import org.firstinspires.ftc.teamcode.subsystems.States.StateManager;

public class OuttakeSlide {
    private static final int slidesnuetral = -110;
    private static final int slidesSpeci = -5;
    private static final int slidesLatchOff = -174;
    private static final int slidesup = -850;

    private DcMotor outtakeSlide;
    private double outtakepos;
    private double f; //for delays
    public OuttakeSlide(HardwareMap hardwareMap){
        outtakeSlide =hardwareMap.get(DcMotor.class, "slides");// outtake, EPS 0 "AXONMAX"
    }

    public void Initalize()
    {
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveToNeutralPos(){
        outtakeSlide.setTargetPosition(slidesnuetral);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void TeleopSample(Gamepad gamepad1, Gamepad gamepad2, StateManager stateManager,
                             OuttakeArm outtakeArm, OutakeClaw outakeClaw, Telemetry telemetry) {
        TeleopSample(gamepad1, gamepad2, stateManager, outtakeArm, outakeClaw, telemetry, true);
    }

    public void TeleopSample(Gamepad gamepad1, Gamepad gamepad2, StateManager stateManager,
                             OuttakeArm outtakeArm, OutakeClaw outakeClaw, Telemetry telemetry, boolean showTelemetry){ //Code to be run in Teleop Mode void Loop at top level
        outtakeSlide.setPower(1);
        if (gamepad2.right_stick_y > 0) {
            outtakeSlide.setTargetPosition((int) (outtakeSlide.getCurrentPosition() + (100 * gamepad2.right_stick_y)));
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //slides.setPower(gamepad2.right_stick_y );
        }
        if (gamepad2.right_stick_y < 0) {
            outtakeSlide.setTargetPosition((int) (outtakeSlide.getCurrentPosition() - (100 * gamepad2.right_stick_y)));
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //slides.setPower(gamepad2.right_stick_y);
        }
        if(gamepad1.y){
            outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // sample slide up
        if (gamepad2.dpad_up) {
            outtakeSlide.setTargetPosition(slidesup);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlide.setPower(1);
            //c1=1;
            stateManager.setSampleSlideUpState(SampleSlideUpState.SAMPLE_SLIDE_GOING_UP);
        }
        if (stateManager.getSampleSlideUpState()==SampleSlideUpState.SAMPLE_SLIDE_GOING_UP
                && Math.abs(outtakeSlide.getCurrentPosition()-slidesup)<=100) {
            //c1=0;
            stateManager.setSampleSlideUpState(SampleSlideUpState.SAMPLE_SLIDE_UP_INIT);
//            rot2pos=rot2out;
//            rotation2.setPosition(rot2out);
            outtakeArm.SetPosToOut();
            outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // sample slide down
        if (gamepad2.dpad_down) {
            //c2=1;
            stateManager.setSampleSlideDownState(SampleSlideDownState.SAMPLE_SLIDE_DOWN_START);
        }
        if(stateManager.getSampleSlideDownState() ==SampleSlideDownState.SAMPLE_SLIDE_DOWN_START){
            if(gamepad2.dpad_down){
//                claw2pos=claw2open;
//                claw2.setPosition(claw2pos);
                outakeClaw.OpenClaw();
            }else{
                //c2=2;
                stateManager.setSampleSlideDownState(SampleSlideDownState.SAMPLE_SLIDE_DOWN_DONE);
            }
        }
        if (stateManager.getSampleSlideDownState()==SampleSlideDownState.SAMPLE_SLIDE_DOWN_DONE){
//            rot2pos=rot2down;
//            rotation2.setPosition(rot2pos);
            outtakeArm.SetPosToDown();
            outtakeSlide.setTargetPosition(slidesnuetral);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlide.setPower(1);
//            claw2pos=claw2close;
//            claw2.setPosition(claw2pos);
            outakeClaw.CloseClaw();
            //c2=0;
            stateManager.setSampleSlideDownState(SampleSlideDownState.SAMPLE_SLIDE_DOWN_INIT);
        }

        if(showTelemetry)
            telemetry.addData("outtake", outtakeSlide.getCurrentPosition());
    }

    public void TeleopSpecimen(Gamepad gamepad2, StateManager stateManager, OuttakeArm outtakeArm,
                               OutakeClaw outakeClaw, Telemetry telemetry) {
        TeleopSpecimen(gamepad2, stateManager, outtakeArm, outakeClaw, telemetry, true);
    }

    public void TeleopSpecimen(Gamepad gamepad2, StateManager stateManager, OuttakeArm outtakeArm,
                               OutakeClaw outakeClaw, Telemetry telemetry, boolean showTelemetry){
        outtakeArm.AdjustRotatePosPerGamepadTrigger(gamepad2);

        if(gamepad2.dpad_right && stateManager.getSpecimenTransferState() == SpecimenTransferState.SPECIMEN_TRANSFER_INIT){
            //rot2pos= rot2wall;
            outtakeArm.SetPosToWall();
            //b1 = 1;
            stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_ARM_GOING_TO_WALL);
        }
        if(stateManager.getSpecimenTransferState()==SpecimenTransferState.SPECIMEN_ARM_GOING_TO_WALL){
            f+=0.025;
            if(f>=1){
                //b1=2;
                stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_ARM_TO_WALL_DONE);
            }
        }
        if(stateManager.getSpecimenTransferState()==SpecimenTransferState.SPECIMEN_ARM_TO_WALL_DONE){
            f=0;
            outtakeSlide.setTargetPosition(-3);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            claw2pos = claw2open;
//            claw2.setPosition(claw2pos);
            outakeClaw.OpenClaw();
            //b1=3;
            stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_ARM_TO_WALL_CLAW_OPEN);
        }
        if(stateManager.getSpecimenTransferState()==SpecimenTransferState.SPECIMEN_ARM_TO_WALL_CLAW_OPEN){
            if(gamepad2.dpad_right){
//                claw2pos = claw2open;
//                claw2.setPosition(claw2pos);
                outakeClaw.OpenClaw();
            }else{
//                claw2pos = claw2close;
//                claw2.setPosition(claw2pos);
                outakeClaw.CloseClaw();
                //b1=4;
                stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_ARM_TO_WALL_CLAW_CLOSING);
            }
        }
        if(stateManager.getSpecimenTransferState()==SpecimenTransferState.SPECIMEN_ARM_TO_WALL_CLAW_CLOSING){
            f+=0.025;
            if(f>=0.2){
                //b1=5;
                stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_ARM_TO_WALL_CLAW_CLOSED);
                f=0;
            }
        }
        if(stateManager.getSpecimenTransferState()==SpecimenTransferState.SPECIMEN_ARM_TO_WALL_CLAW_CLOSED){
            outtakeSlide.setTargetPosition(slidesLatchOff);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(Math.abs(outtakeSlide.getCurrentPosition()-slidesLatchOff)<=10 ){
                //b1=6;
                stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_ARM_TO_WALL_LATCH_OFF);
            }

        }
        if(stateManager.getSpecimenTransferState()==SpecimenTransferState.SPECIMEN_ARM_TO_WALL_LATCH_OFF){
            //if(Math.abs(slides.getCurrentPosition()-slidesLatchOff)<=10 ){
//            rot2pos= rot2speci;
//            rotation2.setPosition(rot2pos);
            outtakeArm.SetPosToSpecimen();
            //}
            //b1=7;
            stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_ARM_ROTATING_TO_FIELD);
        }
        if(stateManager.getSpecimenTransferState()==SpecimenTransferState.SPECIMEN_ARM_ROTATING_TO_FIELD){
            f+=0.025;
            if(f>=0.5){
                //b1=8;
                stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_ARM_TO_FIELD_LATCH_ON);
                f=0;
            }
        }
        if(stateManager.getSpecimenTransferState()==SpecimenTransferState.SPECIMEN_ARM_TO_FIELD_LATCH_ON){
            outtakeSlide.setTargetPosition(slidesSpeci);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //b1=0;
            stateManager.setSpecimenTransferState(SpecimenTransferState.SPECIMEN_TRANSFER_INIT);
        }

        outtakeArm.GoToPosition();
        if(showTelemetry)
            telemetry.addData("b1", stateManager.getSpecimenTransferState());
    }

    public void setPosition(double value) {
        outtakepos = value;
        //outtakeSlide.setPosition(value);
    }

    public void goToNeutralPosition(){
        outtakeSlide.setTargetPosition(slidesnuetral);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlide.setPower(0.8);
    }
}
