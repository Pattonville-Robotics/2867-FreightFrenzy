package org.firstinspires.ftc.teamcode;

// Drop the block in da hub, go to ducks and spin one duck, and then park in storage.

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;
import org.firstinspires.ftc.teamcode.dependencies.Arm;
import org.firstinspires.ftc.teamcode.dependencies.ColorSensor;
import org.firstinspires.ftc.teamcode.dependencies.CommonParameters;
import org.firstinspires.ftc.teamcode.dependencies.ScoopArm;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;

public class StorageParkDuck {
    public static void run(LinearOpMode linearOp, AllianceSide allianceSide) {
        HardwareMap hardwareMap = linearOp.hardwareMap;
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right");
        DcMotor spinny = hardwareMap.dcMotor.get("spinny");
        Arm arm = new ScoopArm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));
        ColorSensor colorSensor = new ColorSensor("Webcam", hardwareMap, linearOp);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        TwoWheelEncoder encoder = new TwoWheelEncoder(leftDrive, rightDrive, imu, CommonParameters.FREIGHT_ROBOT, linearOp);

        linearOp.waitForStart();

        // some variables to simplify stuff
        boolean isRedSide = allianceSide == AllianceSide.RED;
        encoder.storeInitialAngle();

        // Use camera frame to determine arm position
        Arm.armPosition armPos;
        if (colorSensor.isRegionGreen(2)){
            armPos = Arm.armPosition.THREE;
        } else if (colorSensor.isRegionGreen(1)){
            armPos = Arm.armPosition.TWO;
        } else {
            armPos = Arm.armPosition.ONE;
        }

        // Move forward slightly before turning and bring arm up
        arm.moveToPosition(armPos, 0.55);
        encoder.moveInches(5, 0.5);

        // Turn towards the shipping hub and move to it
        rotationalDirection towardsHub = isRedSide ? rotationalDirection.CLOCKWISE : rotationalDirection.COUNTERCLOCKWISE;
        encoder.rotateDegrees(towardsHub, 33.5, 0.35); // TODO: use blue angle in comp
        encoder.moveInches(DcMotorSimple.Direction.FORWARD, 23, 0.5);

        // Spit out the block
        arm.startOuttake();
        linearOp.sleep(3600);
        arm.stopHand();

        // Back up slightly, turn towards carousel and back up into it (slow downs when near it)
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 6.25, 0.55);
        arm.moveToPosition(Arm.armPosition.ONE, 0.5);
        encoder.rotateDegrees(towardsHub, isRedSide ? 39 : 29.5, 0.3);
        encoder.moveInches(DcMotorSimple.Direction.REVERSE,
                isRedSide ? 30.2 : 30,
                0.6);
        encoder.moveInches(DcMotorSimple.Direction.REVERSE,
                7,
                0.3);

        // (Blue only) Spin towards the carousel and spin back afterwards
        if (!isRedSide){
            encoder.rotateDegrees(rotationalDirection.CLOCKWISE, 25);
        }

        // Spin the carousel
        linearOp.sleep(200);
        spinny.setPower(isRedSide ? -0.2 : 0.2);
        linearOp.sleep(3800);
        spinny.setPower(0);
        linearOp.sleep(200);

        // (Blue only) Undo blue rotation from earlier
        if (!isRedSide){
            encoder.rotateDegrees(rotationalDirection.COUNTERCLOCKWISE, 25);
        }

        // Move forward, turn towards the storage unit (or whatever its called).
        encoder.moveInches(8, 0.4);
        rotationalDirection towardsStorage = isRedSide ? rotationalDirection.COUNTERCLOCKWISE : rotationalDirection.CLOCKWISE;
        encoder.rotateDegrees(towardsStorage, isRedSide ? 65 : 54.5, 0.5);
        encoder.moveInches(19.84, 0.5);
        arm.moveToPosition(Arm.armPosition.NEUTRAL, 0.5);

        // Once lined up vertically, turn 90 degrees and back up, for greater chance of being fully within
        encoder.rotateDegrees(towardsHub, 90);
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 11.25, 0.2);
    }

}

