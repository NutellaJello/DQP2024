package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Claws.OutakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Rotations.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.slides.OuttakeSlide;

public class OuttakeSys {
    private OuttakeSlide outtakeSlide;
    private OuttakeArm outtakeArm;
    private OutakeClaw outtakeClaw;

    public OuttakeSys(HardwareMap hardwareMap) {
        outtakeSlide = new OuttakeSlide(hardwareMap);
        outtakeArm = new OuttakeArm(hardwareMap);
        outtakeClaw = new OutakeClaw(hardwareMap);
    }

    public void Initialize()
    {
        outtakeSlide.Initalize();
        outtakeArm.Initialize();
        outtakeClaw.Intialize();
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry) {
        Teleop(gamepad, telemetry, false);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean showTelemetry){

    }
}
