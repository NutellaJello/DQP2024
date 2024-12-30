package org.firstinspires.ftc.teamcode.subsystems.Claws;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class OutakeClaw extends ClawBase{
    private static final double claw2pos=0.347;
    private static final double claw2close = 0.347;
    private static final double claw2open = 0.1;
    public OutakeClaw(HardwareMap hardwareMap){
        super(hardwareMap, "claw2");
    }

    public void Intialize()
    {
        // outtake claw has no initialization pos??
    }
}