package org.firstinspires.ftc.teamcode;

// A two-player teleop.
// Player 1 controls only driving,
// while player 2 controls the arm, claw, wrist and carousel spinner.

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dependencies.Arm.ArmPosition;
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

        currentArmPosition = ArmPosition.NEUTRAL;
    }

    long timeSinceCycleStart;
    double maxSpeed = 1.0;

    ArmPosition currentArmPosition;

    public void loop() {
        telemetry.clearAll();

        // ==== GAMEPAD 1 - Driving (Alex)
        // Store power from joystick values
        double leftInput = gamepad1.right_stick_x;
        double rightInput = gamepad1.left_stick_y;
        double leftSpd = leftInput + rightInput;
        double rightSpd = leftInput - rightInput;

        // Slow if bumpers pushed
        if(gamepad1.right_bumper && gamepad1.left_bumper){
            maxSpeed = 0.125; // L+R  = 12.5% speed
        } else if (gamepad1.right_bumper) {
            maxSpeed = 0.25; // R    = 25% speed
        } else if (gamepad1.left_bumper) {
            maxSpeed = 0.5; // L    = 50% speed
        } else {
            maxSpeed = 1.0; // None = 100% speed
        }

        // Set drive power quadratically and scale with maxSpeed
        left.setPower(leftSpd * Math.abs(leftSpd) * maxSpeed);
        right.setPower(rightSpd * Math.abs(rightSpd) * maxSpeed);


        // ==== GAMEPAD 2 - Arm (Jack)
        // If left bumper is held while a button is pressed, the arm is moved to its backwards variant
        boolean backArm = gamepad2.left_bumper;

        // Move to appropriate location based on button pressed and left bumper state
        if(gamepad2.a){
            currentArmPosition = backArm ? ArmPosition.BACK_NEUTRAL : ArmPosition.NEUTRAL;
        }else if(gamepad2.x){
            currentArmPosition = backArm ? ArmPosition.BACK_ONE : ArmPosition.ONE;
        }else if(gamepad2.y){
            currentArmPosition = backArm ? ArmPosition.BACK_TWO : ArmPosition.TWO;
        }else if(gamepad2.b){
            currentArmPosition = backArm ? ArmPosition.BACK_THREE : ArmPosition.THREE;
        }else if (gamepad2.left_stick_button || gamepad2.right_stick_button){
            currentArmPosition = ArmPosition.CAP;
        }

        // Tuned up/down based on left joystick's Y value
        arm.moveToPosition(currentArmPosition.ticks + (int)(gamepad2.left_stick_y * 30), 0.7);


        // Claw - Right trigger to close hand, left trigger to open hand.
        arm.setHandPower(gamepad2.right_trigger - gamepad2.left_trigger);


        // Wrist - Move with right joystick up&down
        arm.setWristPower(Math.abs(gamepad2.right_stick_y));


        // Spinny - When not being held, time start is set to current time millis.
        // When the button is held, the timer is not reset and allowed to run.
        // Whenever the current time exceeds the timer start value plus a certain amount,
        // the wheel will spin fast to boost the duck off the carousel against the metal bar.
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
        telemetry.addData("hand power: ", arm.getClaw().getPower());
        telemetry.addData("wrist power: ", arm.getWrist().getPower());
        telemetry.update();
    }

    private boolean isFastSpeed(){
        return System.currentTimeMillis() > timeSinceCycleStart+1800 || gamepad1.left_bumper;
    }
}
