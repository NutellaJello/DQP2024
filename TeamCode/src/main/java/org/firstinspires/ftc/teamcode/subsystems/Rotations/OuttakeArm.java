package org.firstinspires.ftc.teamcode.subsystems.Rotations;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArm {
    //outtake arm rotation
    private static final double rot2pos=0.7;
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
        outTakeArm.setPosition(0.7);
    }
}
