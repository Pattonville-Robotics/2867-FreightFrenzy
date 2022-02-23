package org.firstinspires.ftc.teamcode;

// Teleop for one player only. (Updated to work with claw wrist arm.)
// this is the secondary teleop, the primary one we will use is two player teleop
// but this exists in case we need to use it

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.dependencies.Arm.ArmPosition;
import org.firstinspires.ftc.teamcode.dependencies.ClawWithWristArm;

@TeleOp(name="Freight TeleOp", group="TeleOp")

public class FreightTeleOp extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor spinny;
    ClawWithWristArm arm;
    BNO055IMU imu;
    DistanceSensor dist;

    public static double ARM_POWER = 0.3;

    public void init() {
        dist = hardwareMap.get(DistanceSensor.class, "distance");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        spinny = hardwareMap.dcMotor.get("spinny");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        arm = new ClawWithWristArm(
                hardwareMap.get(DcMotor.class, "arm"),
                hardwareMap.get(CRServo.class, "scoop"),
                hardwareMap.get(Servo.class, "wrist"));

        // IMU, used for orientation
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    long timeSinceCycleStart;
    double maxSpeed = 1.0;

    private boolean isFastSpinnySpeed(){
        return System.currentTimeMillis() > timeSinceCycleStart+1800 || gamepad1.left_bumper;
    }

    public void loop() {
        // drive
        telemetry.clearAll();
        double leftInput = gamepad1.right_stick_x;
        double rightInput = gamepad1.left_stick_y;

        double leftSpd = leftInput + rightInput;
        double rightSpd = leftInput - rightInput;

        //   Slow if bumpers pushed
        if(gamepad1.right_bumper && gamepad1.left_bumper){
            maxSpeed = 0.1;
        } else if (gamepad1.right_bumper) {
            maxSpeed = 0.2;
        } else if (gamepad1.left_bumper) {
            maxSpeed = 0.5;
        } else {
            maxSpeed = 1.0;
        }

        left.setPower(leftSpd * Math.abs(leftSpd) * maxSpeed);
        right.setPower(rightSpd * Math.abs(rightSpd) * maxSpeed);

        // arm
        if(gamepad1.a){
            arm.moveToPosition(gamepad1.back ? ArmPosition.BACK_NEUTRAL : ArmPosition.NEUTRAL, 0.7);
        }else if(gamepad1.x){
            arm.moveToPosition(gamepad1.back ? ArmPosition.BACK_ONE : ArmPosition.ONE, 0.7);
        }else if(gamepad1.y){
            arm.moveToPosition(gamepad1.back ? ArmPosition.BACK_TWO : ArmPosition.TWO, 0.7);
        }else if(gamepad1.b){
            arm.moveToPosition(gamepad1.back ? ArmPosition.BACK_THREE : ArmPosition.THREE, 0.7);
        }else if(gamepad1.right_stick_button || gamepad1.start){
            arm.moveToPosition(ArmPosition.CAP, 0.7);
        }

        // hand
//        if(gamepad1.left_trigger>0){
//            arm.closeHand();
//        }else if(gamepad1.right_trigger>0){
//            arm.openHand();
//        }else{
//            arm.stopHand();
//        }
        arm.setHandPower(gamepad1.left_trigger - gamepad1.right_trigger);

        // wrist
        if(gamepad1.dpad_left){
            arm.wristUp();
        }else if(gamepad1.dpad_right){
            arm.wristDown();
        }else if(gamepad1.start){
            arm.wristCap();
        }

        // spinny
        if(gamepad1.dpad_up){
            spinny.setPower(isFastSpinnySpeed() ? -0.6 : -0.22);
        }
        else if(gamepad1.dpad_down){
            spinny.setPower(isFastSpinnySpeed() ? 0.6 : 0.22);
        } else {
            timeSinceCycleStart = System.currentTimeMillis();
            spinny.setPower(0);
        }

        // telemetry
        telemetry.addData("left_trigger: ", gamepad1.left_trigger);
        telemetry.addData("right_trigger: ", gamepad1.right_trigger);
        telemetry.addData("arm position: ", arm.getArmMotor().getCurrentPosition());
        telemetry.addData("hand power: ", arm.getClaw().getPower());
        telemetry.addData("wrist position: ", arm.getWrist().getPosition());
        telemetry.addData("distance: ", dist.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
