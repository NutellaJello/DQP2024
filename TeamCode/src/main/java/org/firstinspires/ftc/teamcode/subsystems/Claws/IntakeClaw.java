package org.firstinspires.ftc.teamcode.subsystems.Claws;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeClaw{
    private static double clawpos=0.47;
    private static double clawclose = 0.06;
    private static double clawopen = 0.45;

    private Servo intakeClaw;
    public IntakeClaw(HardwareMap hardwareMap){
        intakeClaw =hardwareMap.get(Servo.class, "claw"); // EPS 1
    }

    public void Initialize(){
        this.OpenClaw();
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry) {
        Teleop(gamepad2, telemetry, true);
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry, boolean showTelemetry) {
        if (gamepad2.x) {
            clawpos=clawclose;//0.08 close
            intakeClaw.setPosition(clawpos);
        }
        if (gamepad2.y) {
            clawpos=clawopen;//-0.47
            intakeClaw.setPosition(clawpos);
        }
        if(showTelemetry)
            telemetry.addData("claw", clawpos);
    }

    public void OpenClaw(){
        clawpos = clawopen;
        intakeClaw.setPosition(clawpos);
    }
    public void CloseClaw(){
        clawpos = clawclose;
        intakeClaw.setPosition(clawpos);
    }
    public boolean PositionReachedWithTolerance(double tolerance){
        return (intakeClaw.getPosition()-clawpos)<= tolerance;
    }
}
