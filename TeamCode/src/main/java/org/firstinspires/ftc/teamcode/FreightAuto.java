package org.firstinspires.ftc.teamcode;

// This file contains the code these 4 files run with different allianceSides and alliancePositions:
// FreightAuto_RedLeft
// FreightAuto_RedRight
// FreightAuto_BlueLeft
// FreightAuto_BlueRight
// - jack

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.dependencies.AlliancePosition;
import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;
import org.firstinspires.ftc.teamcode.dependencies.Arm;
import org.firstinspires.ftc.teamcode.dependencies.ColorSensor;
import org.firstinspires.ftc.teamcode.dependencies.CommonParameters;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;

public class FreightAuto {
    private static DcMotor leftDrive;
    private static DcMotor rightDrive;
    private static Arm arm;
    private static ColorSensor colorSensor;
    private static BNO055IMU imu;

    private static TwoWheelEncoder encoder;

    public static void run(LinearOpMode linearOp, AllianceSide allianceSide, AlliancePosition alliancePosition){
        // Declare any local / helper variables here
        // Our initialization code should go here
        HardwareMap hardwareMap = linearOp.hardwareMap;
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        arm = new Arm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));
        colorSensor = new ColorSensor("Webcam", hardwareMap, linearOp);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        encoder = new TwoWheelEncoder(leftDrive, rightDrive, imu, CommonParameters.FREIGHT_ROBOT, linearOp);

        linearOp.waitForStart();

        //Insert camera code here
        // for now, assume the duck is in the middle
        Arm.armPosition armPos = Arm.armPosition.TWO;

        // Move the arm to the appropriate height
        arm.moveToPosition(armPos, 0.7);
        encoder.moveInches(4);

        // Move to the hub (This will run while the arm is moving to save time)
        rotationalDirection towardsHub;
        rotationalDirection awayFromHub;
        if (alliancePosition == AlliancePosition.LEFT){
            towardsHub = rotationalDirection.CLOCKWISE;
            awayFromHub = rotationalDirection.COUNTERCLOCKWISE;
        } else {
            towardsHub = rotationalDirection.COUNTERCLOCKWISE;
            awayFromHub = rotationalDirection.CLOCKWISE;
        }
        encoder.rotateDegrees(towardsHub, 90);
        encoder.moveInches(15);
        encoder.rotateDegrees(awayFromHub, 90);
        encoder.moveInches(17);

        // Spit out the block
        arm.startOuttake();
        linearOp.sleep(2000);
        arm.stopHand();

        // Back up and lower the arm
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 17, 0.7);

        arm.moveToPosition(Arm.armPosition.NEUTRAL, 0.7);
        rotationalDirection towardsDepot;
        rotationalDirection awayFromDepot;
        if (allianceSide == AllianceSide.RED){
            towardsDepot = rotationalDirection.CLOCKWISE;
            awayFromDepot = rotationalDirection.COUNTERCLOCKWISE;
        } else {
            towardsDepot = rotationalDirection.COUNTERCLOCKWISE;
            awayFromDepot = rotationalDirection.CLOCKWISE;
        }
        encoder.rotateDegrees(towardsDepot, 96);
        encoder.moveInches(23);
        encoder.rotateDegrees(awayFromDepot, 6);
        encoder.moveInches(26);
    }

}

