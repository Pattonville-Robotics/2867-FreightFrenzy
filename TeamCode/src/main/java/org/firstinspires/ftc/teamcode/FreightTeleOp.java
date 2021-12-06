package org.firstinspires.ftc.teamcode;

// hey guys justin here i just added the
// very based spinny code ("DcMotor spinny;")
// -justin, creator of the very based spinny code
//                     (December 6th, 2:48 PM)

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.dependencies.Arm;
import org.firstinspires.ftc.teamcode.dependencies.Arm.armPosition;

@TeleOp(name="Freight TeleOp", group="TeleOp")

public class FreightTeleOp extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor spinny;
    Arm arm;
    BNO055IMU imu;

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        spinny = hardwareMap.dcMotor.get("spinny");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        arm = new Arm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));

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

        double leftInput = gamepad1.right_stick_x;
        double rightInput = gamepad1.left_stick_y;


        double leftSpd = leftInput + rightInput;
        double rightSpd = leftInput - rightInput;
        if((Math.abs(leftSpd)==1)&&(Math.abs(rightSpd)==1)) {
            left.setPower(leftSpd);
            right.setPower(rightSpd);
        }else{
            left.setPower(leftSpd * 0.5);
            right.setPower(rightSpd * 0.5);
        }

        if(gamepad1.dpad_up){
            spinny.setPower(1);
        }
        else{
            spinny.setPower(0);
        }

        if(gamepad1.a){
            arm.moveToPosition(armPosition.NEUTRAL, 0.7);
        }else if(gamepad1.x){
            arm.moveToPosition(armPosition.ONE, 0.7);
        }else if(gamepad1.y){
            arm.moveToPosition(armPosition.TWO, 0.7);
        }else if(gamepad1.b){
            arm.moveToPosition(armPosition.THREE, 0.7);
        }

        if(gamepad1.left_trigger>0){
            arm.startIntake();
        }else if(gamepad1.right_trigger>0){
            arm.startOuttake();
        }else{
            arm.stopHand();
        }
        telemetry.addData("A: ", gamepad1.a);
        telemetry.addData("B: ", gamepad1.b);
        telemetry.addData("X: ", gamepad1.x);
        telemetry.addData("Y: ", gamepad1.y);
        telemetry.addData("left_trigger: ", gamepad1.left_trigger);
        telemetry.addData("right_trigger: ", gamepad1.right_trigger);
        telemetry.addData("encoderPos: ", arm.getArmMotor().getCurrentPosition());
        telemetry.update();
    }
}
