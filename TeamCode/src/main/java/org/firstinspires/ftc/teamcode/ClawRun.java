package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ClawRun", group="TeleOp")
public class ClawRun extends OpMode {
    Servo wrist;
    CRServo claw;
    DcMotor arm;
    public void init(){
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(CRServo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm");
    }
    public void loop() {
        telemetry.clearAll();
        wrist.setPosition(Math.abs(gamepad1.left_stick_y));
        claw.setPower(gamepad1.right_stick_y*0.5);
        telemetry.addData("armPos: ", arm.getCurrentPosition());
        telemetry.addData("wristPos: ", wrist.getPosition());
    }
}
