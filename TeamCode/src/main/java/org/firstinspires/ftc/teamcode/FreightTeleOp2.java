package org.firstinspires.ftc.teamcode;

// hey guys justin here i just added the
// very based spinny code ("DcMotor spinny;")
// -justin, creator of the very based spinny code
//                     (December 6th, 2:48 PM)

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dependencies.Arm;
import org.firstinspires.ftc.teamcode.dependencies.Arm.armPosition;
import org.firstinspires.ftc.teamcode.dependencies.ScoopArm;

@TeleOp(name="Freight TeleOp 2", group="TeleOp")

public class FreightTeleOp2 extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor spinny;
    ScoopArm arm;
    BNO055IMU imu;

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        spinny = hardwareMap.dcMotor.get("spinny");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        arm = new ScoopArm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));

        // IMU, used for orientation
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        currentArmPosition = armPosition.NEUTRAL;
    }

    long timeSinceCycleStart;
    double maxSpeed = 1.0;

    armPosition currentArmPosition;

    public void loop() {
        telemetry.clearAll();
        double leftInput = gamepad1.right_stick_x;
        double rightInput = gamepad1.left_stick_y;

        double leftSpd = leftInput + rightInput;
        double rightSpd = leftInput - rightInput;

        // Slow if right bumper pushed
        if(gamepad1.right_bumper){
            maxSpeed = 0.2;
        } else {
            maxSpeed = 1.0;
        }

        // Speed cap
//        left.setPower(Math.min(leftSpd * Math.abs(leftSpd), maxSpeed));
//        right.setPower(Math.min(rightSpd * Math.abs(rightSpd), maxSpeed));

        // direct scale
        left.setPower(leftSpd * Math.abs(leftSpd) * maxSpeed);
        right.setPower(rightSpd * Math.abs(rightSpd) * maxSpeed);

        if(gamepad2.dpad_up){
            spinny.setPower(isFastSpeed() ? -0.6 : -0.22);
        }
        else if(gamepad2.dpad_down){
            spinny.setPower(isFastSpeed() ? 0.6 : 0.22);
        } else {
            timeSinceCycleStart = System.currentTimeMillis();
            spinny.setPower(0);
        }

        if(gamepad2.a){
            currentArmPosition = armPosition.NEUTRAL;
        }else if(gamepad2.x){
            currentArmPosition = armPosition.ONE;
        }else if(gamepad2.y){
            currentArmPosition = armPosition.TWO;
        }else if(gamepad2.b){
            currentArmPosition = armPosition.THREE;
        }
        moveArm();

//        if(gamepad2.left_trigger>0){
//            arm.startIntake();
//        }else if(gamepad2.right_trigger>0){
//            arm.startOuttake();
//        }else{
//            arm.stopHand();
//        }

        // Add right trigger to power and subtract left trigger from power (WARNIGN: CURSED)
        arm.setScoopPower(gamepad2.right_trigger - gamepad2.left_trigger);

        telemetry.addData("A: ", gamepad1.a);
        telemetry.addData("B: ", gamepad1.b);
        telemetry.addData("X: ", gamepad1.x);
        telemetry.addData("Y: ", gamepad1.y);
        telemetry.addData("left_trigger: ", gamepad1.left_trigger);
        telemetry.addData("right_trigger: ", gamepad1.right_trigger);
        telemetry.addData("encoderPos: ", arm.getArmMotor().getCurrentPosition());
        //telemetry.addData("cycleRunning: ", ducksSpinning);
        telemetry.update();
    }

    private boolean isFastSpeed(){
        return System.currentTimeMillis() > timeSinceCycleStart+1800 || gamepad1.left_bumper;
    }

    private void moveArm(){
        int ticks = currentArmPosition.ticks;
        if (gamepad2.left_bumper) ticks -= 5;
        if (gamepad2.right_bumper) ticks += 5;
        arm.moveToPosition(ticks, 0.7);
    }
}
