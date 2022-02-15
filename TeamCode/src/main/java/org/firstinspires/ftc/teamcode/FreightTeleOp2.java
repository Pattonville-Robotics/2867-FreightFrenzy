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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dependencies.Arm.armPosition;
import org.firstinspires.ftc.teamcode.dependencies.ClawWithWristArm;

@TeleOp(name="Freight TeleOp 2 Player", group="TeleOp")

public class FreightTeleOp2 extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor spinny;
    ClawWithWristArm arm;
    BNO055IMU imu;

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        spinny = hardwareMap.dcMotor.get("spinny");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        arm = new ClawWithWristArm(
                hardwareMap.get(DcMotor.class, "arm"),
                hardwareMap.get(CRServo.class, "scoop"),
                hardwareMap.get(CRServo.class, "wrist"));

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

        // ==== GAMEPAD 1 - Driving (Alex)

        // Store power from joystick values
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

        // Set power quadratically and scale with maxSpeed
        left.setPower(leftSpd * Math.abs(leftSpd) * maxSpeed);
        right.setPower(rightSpd * Math.abs(rightSpd) * maxSpeed);



        // ==== GAMEPAD 2 - Arm (Jack)

        // Set arm base level if a button is pushed, bumpers will make it move to the backward positions
        boolean backArm = gamepad2.left_bumper || gamepad2.right_bumper;
        if(gamepad2.a){
            currentArmPosition = backArm ? armPosition.BACK_NEUTRAL : armPosition.NEUTRAL;
        }else if(gamepad2.x){
            currentArmPosition = backArm ? armPosition.BACK_ONE : armPosition.ONE;
        }else if(gamepad2.y){
            currentArmPosition = backArm ? armPosition.BACK_TWO : armPosition.TWO;
        }else if(gamepad2.b){
            currentArmPosition = backArm ? armPosition.BACK_THREE : armPosition.THREE;
        }

        // Get armpos ticks and tune up/down based on left joystick's Y value
        arm.moveToPosition(currentArmPosition.ticks + (int)(gamepad2.left_stick_y * 15), 0.7);


        // Claw - Add right trigger to power and subtract left trigger from power (WARNIGN: CURSED)
        arm.setClawPower(gamepad2.right_trigger - gamepad2.left_trigger);

        // Wrist - Move with right Y
        arm.setWristPower(Math.abs(gamepad2.right_stick_y));

        // Spinny moment
        if(gamepad2.dpad_up){
            spinny.setPower(isFastSpeed() ? -0.6 : -0.22);
        }
        else if(gamepad2.dpad_down){
            spinny.setPower(isFastSpeed() ? 0.6 : 0.22);
        } else {
            timeSinceCycleStart = System.currentTimeMillis();
            spinny.setPower(0);
        }


        // == Telemetry
        telemetry.addLine(String.format("A: %s, B: %s, X: %s, Y: %s",
                gamepad1.a, gamepad2.b, gamepad2.x, gamepad2.y));
        telemetry.addData("left_trigger: ", gamepad2.left_trigger);
        telemetry.addData("right_trigger: ", gamepad2.right_trigger);
        telemetry.addData("arm position: ", arm.getArmMotor().getCurrentPosition());
        telemetry.addData("hand power: ", arm.getHandServo().getPower());
        telemetry.addData("wrist power: ", arm.getWrist().getPower());
        //telemetry.addData("cycleRunning: ", ducksSpinning);
        telemetry.update();
    }

    private boolean isFastSpeed(){
        return System.currentTimeMillis() > timeSinceCycleStart+1800 || gamepad1.left_bumper;
    }
}
