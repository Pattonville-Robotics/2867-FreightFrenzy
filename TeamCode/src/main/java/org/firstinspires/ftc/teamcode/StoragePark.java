package org.firstinspires.ftc.teamcode;

// Uses camera to drop block in hub, then parks in the storage unit.

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
import org.firstinspires.ftc.teamcode.dependencies.ClawWithWristArm;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;

public class StoragePark {
    public static void run(LinearOpMode linearOp, AllianceSide allianceSide) {
        HardwareMap hardwareMap = linearOp.hardwareMap;
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right");
        Arm arm = new ClawWithWristArm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));
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

        Arm.ArmPosition armPos;
        if (colorSensor.isRegionGreen(2)){
            armPos = Arm.ArmPosition.THREE;
        } else if (colorSensor.isRegionGreen(1)){
            armPos = Arm.ArmPosition.TWO;
        } else {
            armPos = Arm.ArmPosition.ONE;
        }

        arm.moveToPosition(armPos, 0.7);
        encoder.moveInches(5, 0.3);

        rotationalDirection towardsHub = allianceSide == AllianceSide.RED ? rotationalDirection.CLOCKWISE : rotationalDirection.COUNTERCLOCKWISE;
        encoder.rotateDegrees(towardsHub, 34, 0.5);
        encoder.moveInches(DcMotorSimple.Direction.FORWARD, 26, 0.5);

        arm.startOuttake();
        linearOp.sleep(4000);
        arm.stopHand();

        // Move back to start and turn
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 24, 0.5);
        arm.moveToPosition(Arm.ArmPosition.ONE, 0.5);

        rotationalDirection towardsStorage = allianceSide == AllianceSide.RED ? rotationalDirection.COUNTERCLOCKWISE : rotationalDirection.CLOCKWISE;
        encoder.rotateDegrees(towardsStorage, 67, 0.5);
        encoder.moveInches(34, 0.5);
        encoder.rotateDegrees(towardsStorage, 47, 0.5);
    }

}

