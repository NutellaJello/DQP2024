package org.firstinspires.ftc.teamcode.subsystems.Rotations;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.States.SpecimenTransferState;

public class OuttakeArm {
    //outtake arm rotation
    private static double rot2pos=0.7;
    private static final double rot2down = 0.7821;
    private static final double rot2out = 0.06;
    private static final double rot2wall = 0.98;
    private static final double rot2speci = 0.371;

    private Servo outTakeArm;
    public OuttakeArm(HardwareMap hardwareMap){
        outTakeArm =hardwareMap.get(Servo.class, "rotation2");// outtake arm, EPS 5
    }

    public void Initialize()
    {
        outTakeArm.setPosition(rot2pos);
        //outTakeArm.setPosition(0.7);
    }
    public void Teleop(Gamepad gamepad2, Telemetry telemetry){
        Teleop(gamepad2, telemetry, true);
    }
    public void Teleop(Gamepad gamepad2, Telemetry telemetry, boolean showTelemetry)
    {
        if(showTelemetry)
            telemetry.addData("rotation2", rot2pos);
    }

    public void AdjustRotatePosPerGamepadTrigger(Gamepad gamepad2)
    {
        rot2pos-=gamepad2.left_trigger/100;
        rot2pos += gamepad2.right_trigger/100;
        if(rot2pos<0){
            rot2pos=0;
        }
        if(rot2pos >1){
            rot2pos =1;
        }


        if (gamepad2.right_trigger>0) {
            if (rot2pos<0.985){
                rot2pos+=gamepad2.right_trigger/100;
            }
            else{
                rot2pos=0.985;
            }
        }
    }

    public void setPosition(double pos){
        rot2pos = pos;
        outTakeArm.setPosition(rot2pos);
    }
    public void SetPosToWall(){
        rot2pos= rot2wall;
    }
    public void SetPosToSpecimen(){
        rot2pos= rot2speci;
        outTakeArm.setPosition(rot2pos);
    }
    public void SetPosToOut(){
        rot2pos=rot2out;
        outTakeArm.setPosition(rot2out);
    }
    public void SetPosToDown(){
        rot2pos=rot2down;
        outTakeArm.setPosition(rot2pos);
    }
    public void GoToPosition()
    {
        outTakeArm.setPosition(rot2pos);
    }
}
