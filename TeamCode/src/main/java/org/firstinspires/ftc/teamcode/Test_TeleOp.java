package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@TeleOp(name="TeleOp Test", group="TeleOp")

public class Test_TeleOp extends OpMode {
    DcMotor left;
    DcMotor right;
    BNO055IMU imu;

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // IMU, used for orientation
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
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
        telemetry.addData("Orientation: ", imu.getAngularOrientation());
        left.setPower(leftSpd);
        right.setPower(-(rightSpd));
        telemetry.update();
    }
}
