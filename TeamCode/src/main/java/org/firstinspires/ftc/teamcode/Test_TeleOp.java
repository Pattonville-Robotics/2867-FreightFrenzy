package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@TeleOp(name="TeleOp Test", group="TeleOp")
public class Test_TeleOp extends OpMode {
    DcMotor left;
    DcMotor right;

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

    }

    double maxSpeed = 1.0;

    public void loop() {
        right.setPower(-(gamepad1.right_stick_y));
        left.setPower(gamepad1.left_stick_y);
        telemetry.clearAll();
        telemetry.addData("right:",(-(Math.abs(gamepad1.right_stick_y)*gamepad1.right_stick_y)/1/maxSpeed));
        telemetry.addData("left:",(Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y)/1/maxSpeed);
        telemetry.update();
    }
}
