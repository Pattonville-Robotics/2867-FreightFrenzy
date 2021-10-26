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
        telemetry.clearAll();

        double leftInput = gamepad1.left_stick_y;
        double rightInput = gamepad1.right_stick_y;

        double leftSpd = maxSpeed * Math.abs(leftInput) * leftInput;
        double rightSpd = maxSpeed * Math.abs(rightInput) * rightInput;

        telemetry.addData("left:", leftSpd);
        telemetry.addData("right:", rightSpd);

        left.setPower(leftSpd);
        right.setPower(-(rightSpd));
        telemetry.update();
    }
}
