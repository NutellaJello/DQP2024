package org.firstinspires.ftc.teamcode.subsystems.Rotations;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeClawRotation {
    private static double pivotpos=0.53;
    private static double pivotnuetral = 0.53;

    private Servo intakeClawRotation;
    public IntakeClawRotation(HardwareMap hardwareMap){
        intakeClawRotation =hardwareMap.get(Servo.class, "pivot");// intake claw rotation, EPS 2
    }

    public void Initialize()
    {
        //intakeClawRotation.setPosition(0.28);;
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry) {
        Teleop(gamepad2, telemetry, true);
    }

    public void Teleop(Gamepad gamepad2, Telemetry telemetry, boolean showTelemetry) {
        pivotpos-=gamepad2.left_stick_x*0.017;

        if (pivotpos>1) {
            pivotpos=1;
        }
        if (pivotpos<0){
            pivotpos=0;
        }

        intakeClawRotation.setPosition(pivotpos);
        if(showTelemetry)
            telemetry.addData("pivot", pivotpos);
    }

    public void PivotNeutral(){
        pivotpos=pivotnuetral;
        intakeClawRotation.setPosition(pivotpos);
    }
}
