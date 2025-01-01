package org.firstinspires.ftc.teamcode.subsystems.Claws;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutakeClaw {
    private static double claw2pos=0.347;
    private static double claw2close = 0.347;
    private static double claw2open = 0.1;

    private Servo outakeClaw;
    public OutakeClaw(HardwareMap hardwareMap){
        outakeClaw =hardwareMap.get(Servo.class, "claw2"); // EPS 4
    }

    public void Intialize()
    {
        // outtake claw has no initialization pos??
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry) {
        Teleop(gamepad2, telemetry, true);
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry, boolean showTelemetry) {
        if (gamepad2.right_bumper) {

            claw2pos = claw2close;
            outakeClaw.setPosition(claw2pos);
        }
        if (gamepad2.left_bumper) {

            claw2pos=claw2open;
            outakeClaw.setPosition(claw2pos);
        }
        if(showTelemetry)
            telemetry.addData("claw2", claw2pos);
    }

    public void OpenClaw(){
        claw2pos = claw2open;
        outakeClaw.setPosition(claw2pos);
    }

    public void CloseClaw(){
        claw2pos = claw2close;
        outakeClaw.setPosition(claw2pos);
    }
}