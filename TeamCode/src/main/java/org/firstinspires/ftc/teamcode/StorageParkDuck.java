package org.firstinspires.ftc.teamcode;

// Drop the block in da hub, go to ducks and spin one duck, and then park in storage.

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;
import org.firstinspires.ftc.teamcode.dependencies.Arm;
import org.firstinspires.ftc.teamcode.dependencies.ColorSensor;
import org.firstinspires.ftc.teamcode.dependencies.CommonParameters;
import org.firstinspires.ftc.teamcode.dependencies.ClawWithWristArm;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;

public class StorageParkDuck {
    public static void run(LinearOpMode linearOp, AllianceSide allianceSide) {
        HardwareMap hardwareMap = linearOp.hardwareMap;
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right");
        DcMotor spinny = hardwareMap.dcMotor.get("spinny");
        ClawWithWristArm arm = new ClawWithWristArm(
                hardwareMap.get(DcMotor.class, "arm"),
                hardwareMap.get(CRServo.class, "scoop"),
                hardwareMap.get(Servo.class, "wrist"));

//        ColorSensor colorSensor = new ColorSensor("Webcam", hardwareMap, linearOp);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        TwoWheelEncoder encoder = new TwoWheelEncoder(leftDrive, rightDrive, imu, CommonParameters.FREIGHT_ROBOT, linearOp);
//        linearOp.waitForStart();

        // some variables to simplify stuff
        boolean isRedSide = allianceSide == AllianceSide.RED;
//        encoder.storeInitialAngle();

        // Use camera frame to determine arm position
        Arm.ArmPosition armPos;
//        if (colorSensor.isRegionGreen(2)){
//            armPos = Arm.ArmPosition.THREE;
//        } else if (colorSensor.isRegionGreen(1)){
//            armPos = Arm.ArmPosition.TWO;
//        } else {
//            armPos = Arm.ArmPosition.ONE;
//        }
        armPos = Arm.ArmPosition.THREE;

        // Move forward slightly before turning and bring arm up
        arm.moveToPosition(armPos, 0.55);
        encoder.moveInches(5, 0.6);
        arm.wristDown();

        // Turn towards the shipping hub and move to it
        rotationalDirection towardsHub = isRedSide ? rotationalDirection.CLOCKWISE : rotationalDirection.COUNTERCLOCKWISE;
        encoder.rotateDegrees(towardsHub, 33.75, 0.35);
        encoder.moveInches(DcMotorSimple.Direction.FORWARD, 23.25, 0.55);

        // Spit out the block
        linearOp.sleep(2000);
        arm.openHand();

        // Back up slightly, turn towards carousel and back up into it (slow downs when near it)
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 6, 0.65);
        arm.moveToPosition(Arm.ArmPosition.ONE, 0.5);
        encoder.rotateDegrees(towardsHub, isRedSide ? 29.5 : 39.5, 0.4);
        encoder.moveInches(DcMotorSimple.Direction.REVERSE,
                isRedSide ? 30 : 27.5,
                0.725);
        encoder.moveInches(DcMotorSimple.Direction.REVERSE,
                9.25,
                0.25);

        // (Red only) Spin towards the carousel and spin back afterwards
        if (isRedSide){
            encoder.rotateDegrees(rotationalDirection.COUNTERCLOCKWISE, 25);
        }

        // Spin the carousel
        linearOp.sleep(200);
        spinny.setPower(isRedSide ? -0.2 : 0.2);
        linearOp.sleep(3200);
        spinny.setPower(0);
        linearOp.sleep(200);

        // (Red only) Undo red rotation from earlier
        if (isRedSide){
            encoder.rotateDegrees(rotationalDirection.CLOCKWISE, 25);
        }

        // Move forward, turn towards the storage unit (or whatever its called).
        encoder.moveInches(8, 0.5);
        rotationalDirection towardsStorage = isRedSide ? rotationalDirection.COUNTERCLOCKWISE : rotationalDirection.CLOCKWISE;
        encoder.rotateDegrees(towardsStorage, isRedSide ? 58.5 : 54, 0.6);
        encoder.moveInches(19.84, 0.6);
        arm.moveToPosition(Arm.ArmPosition.NEUTRAL, 0.6);
        arm.wristUp();

        // Once lined up vertically, turn 90 degrees and back up
        encoder.rotateDegrees(towardsHub, 90);
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 11, 0.35);
    }

}

