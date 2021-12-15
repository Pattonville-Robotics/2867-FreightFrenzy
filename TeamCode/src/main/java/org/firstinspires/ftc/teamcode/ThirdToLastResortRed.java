package org.firstinspires.ftc.teamcode;

// Drives forward and drops the block, thats it.

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.dependencies.Arm;
import org.firstinspires.ftc.teamcode.dependencies.ColorSensor;
import org.firstinspires.ftc.teamcode.dependencies.CommonParameters;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;


@Autonomous(name="ThirdToLastResortRed", group="Autonomous")
public class ThirdToLastResortRed extends LinearOpMode {

    private DcMotor leftDrive = null;
    private Arm arm;
    private ColorSensor colorSensor;
    private DcMotor rightDrive = null;
    private BNO055IMU imu;
    private TwoWheelEncoder encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare any local / helper variables here
        // Our initialization code should go here
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        arm = new Arm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));
        colorSensor = new ColorSensor("Webcam", hardwareMap, this);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        encoder = new TwoWheelEncoder(leftDrive, rightDrive, imu, CommonParameters.FREIGHT_ROBOT, this);

        waitForStart();

        Arm.armPosition armPos;
        if (colorSensor.isRegionGreen(2)){
            armPos = Arm.armPosition.THREE;
        } else if (colorSensor.isRegionGreen(1)){
            armPos = Arm.armPosition.TWO;
        } else {
            armPos = Arm.armPosition.ONE;
        }

        arm.moveToPosition(armPos, 0.7);

        encoder.moveInches(5, 0.3);
        encoder.rotateDegrees(rotationalDirection.CLOCKWISE, 34, 0.5);
        encoder.moveInches(DcMotorSimple.Direction.FORWARD, 26, 0.5);

        arm.startOuttake();
        sleep(4000);
        arm.stopHand();

        // Move back to start and turn
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 24, 0.5);
        arm.moveToPosition(Arm.armPosition.ONE, 0.5);
        encoder.rotateDegrees(rotationalDirection.COUNTERCLOCKWISE, 67, 0.5);
        encoder.moveInches(34, 0.5);
        encoder.rotateDegrees(rotationalDirection.COUNTERCLOCKWISE, 47, 0.5);
    }

}

