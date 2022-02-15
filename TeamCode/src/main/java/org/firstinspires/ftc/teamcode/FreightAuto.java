package org.firstinspires.ftc.teamcode;

// This file contains the run method these 4 files call with different allianceSides and alliancePositions:
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
import org.firstinspires.ftc.teamcode.dependencies.Arm.armPosition;
import org.firstinspires.ftc.teamcode.dependencies.ColorSensor;
import org.firstinspires.ftc.teamcode.dependencies.CommonParameters;
import org.firstinspires.ftc.teamcode.dependencies.ClawWithCapArm;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;

public class FreightAuto {
    public static void run(LinearOpMode linearOp, AllianceSide allianceSide, AlliancePosition alliancePosition){
        // Declare any local / helper variables here
        // Our initialization code should go here
        HardwareMap hardwareMap = linearOp.hardwareMap;
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right");
        Arm arm = new ClawWithCapArm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));
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

        //Insert camera code here
        // for now, assume the duck is in the middle
        Arm.armPosition armPos;
        if (colorSensor.isRegionGreen(2)){
            armPos = Arm.armPosition.THREE;
        } else if (colorSensor.isRegionGreen(1)){
            armPos = Arm.armPosition.TWO;
        } else {
            armPos = Arm.armPosition.ONE;
        }

        // Move the arm to the appropriate height
        arm.moveToPosition(armPos, 0.35);
        encoder.moveInches(6);

        // Move to the hub (This will run while the arm is moving to save time)
        rotationalDirection towardsHub;
        if (alliancePosition == AlliancePosition.LEFT){
            towardsHub = rotationalDirection.CLOCKWISE;
        } else {
            towardsHub = rotationalDirection.COUNTERCLOCKWISE;
        }
        encoder.rotateDegrees(towardsHub, 34.75, 0.3);
        encoder.moveInches(23);

        // Spit out the block
        arm.startOuttake();
        linearOp.sleep(4000);
        arm.stopHand();

        rotationalDirection towardsDepot;
        rotationalDirection awayFromDepot;
        if (allianceSide == AllianceSide.RED){
            towardsDepot = rotationalDirection.CLOCKWISE;
            awayFromDepot = rotationalDirection.COUNTERCLOCKWISE;
        } else {
            towardsDepot = rotationalDirection.COUNTERCLOCKWISE;
            awayFromDepot = rotationalDirection.CLOCKWISE;
        }
        int rotation1;
        int rotation2;
        int distanceFromDepot;
        if (allianceSide == AllianceSide.RED && alliancePosition == AlliancePosition.RIGHT
        || allianceSide == AllianceSide.BLUE && alliancePosition == AlliancePosition.LEFT ){
            // CLoser to depot
            rotation1 = 33;
            rotation2 = 30;
            distanceFromDepot = 16;
        } else {
            // Further from depot
            rotation1 = 70;
            rotation2 = 18;
            distanceFromDepot = 47;
        }

        // move back halfway, turn 90 degrees and move the rest of the way
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 12, 0.5);
        encoder.rotateDegrees(towardsDepot, 180, 0.5);
        encoder.moveInches(DcMotorSimple.Direction.FORWARD, 15, 0.5);

        //turn towards garage opening and move towards it, then turn towards inside of garage and move into it
        encoder.rotateDegrees(awayFromDepot, rotation1);
        encoder.moveInches(distanceFromDepot);
        encoder.rotateDegrees(awayFromDepot, rotation2);
        encoder.moveInches(20);

        arm.moveToPosition(armPosition.NEUTRAL, 0.4);

    }

}

