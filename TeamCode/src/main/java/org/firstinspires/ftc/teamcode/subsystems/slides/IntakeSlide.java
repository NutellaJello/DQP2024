package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Claws.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Rotations.IntakePitchArm;
import org.firstinspires.ftc.teamcode.subsystems.States.SampleManualState;
import org.firstinspires.ftc.teamcode.subsystems.States.StateManager;

public class IntakeSlide {
    private static double slides2pos=0.4968;
    private static double slides2out = 0.77;
    private static double slides2in= 0.4968;

    private Servo intakeSlide;
    public IntakeSlide(HardwareMap hardwareMap){
        intakeSlide =hardwareMap.get(Servo.class, "slides2");// intake, EPS 0 "AXONMAX"
    }

    public void Teleop(Gamepad gamepad2, StateManager stateManager, IntakePitchArm intakePitchArm,
                       IntakeClaw intakeClaw, Telemetry telemetry) {
        Teleop(gamepad2, stateManager, intakePitchArm, intakeClaw,  telemetry, true);
    }

    public void Teleop(Gamepad gamepad2, StateManager stateManager, IntakePitchArm intakePitchArm, IntakeClaw intakeClaw,
                       Telemetry telemetry, boolean showTelemetry){ //Code to be run in Teleop Mode void Loop at top level
        double idkman= gamepad2.left_stick_y;
        slides2pos -= idkman/500;
        if(slides2pos >0.8){
            slides2pos = 0.8;
        }
        if (slides2pos<slides2in){
            slides2pos = slides2in;
        }
        if(gamepad2.b){
            slides2pos=slides2out;
            intakePitchArm.PitchDown();
        }

        intakeSlide.setPosition(slides2pos);

        // for sample manual teleop
        if(gamepad2.left_stick_button){
            //d1 =1;
            stateManager.setSampleManualState(SampleManualState.SAMPLE_MANUAL_PICKUP);
        }
        if(stateManager.getSampleManualState() == SampleManualState.SAMPLE_MANUAL_PICKUP){
            if(gamepad2.left_stick_button) {
//                clawpos = clawclose;
//                claw.setPosition(clawpos);
                intakeClaw.CloseClaw();
//                rotpos = 0.75;
//                rotation.setPosition(rotpos);
                intakePitchArm.SetPitchPosition(0.75);

            }else{
//                slides2pos = slides2in;
//                slides2.setPosition(slides2pos);
                slides2pos = slides2in;
                intakeSlide.setPosition(slides2pos);
                //d1=0;
                stateManager.setSampleManualState(SampleManualState.SAMPLE_MANUAL_INIT);
            }

        }
        if(showTelemetry) {
            telemetry.addData("slides2", slides2pos);
            //telemetry.addData("rotation", intakePitchArm.GetPitchPosition());
        }

    }
    public void setPosition(double value) {
        slides2pos = value;
        intakeSlide.setPosition(slides2pos);
    }
    public void SlideIn(){
        slides2pos=slides2in;
        intakeSlide.setPosition(slides2pos);
    }
    public void SlideInTweak(){
        slides2pos=slides2in+0.14;
        intakeSlide.setPosition(slides2pos);
    }
    public void SlideOut(){
        slides2pos=slides2out;
        intakeSlide.setPosition(slides2pos);
    }
    public boolean PositionReachedWithTolerance(double tolerance){
        return (intakeSlide.getPosition() - slides2pos) <= tolerance;
    }
}
