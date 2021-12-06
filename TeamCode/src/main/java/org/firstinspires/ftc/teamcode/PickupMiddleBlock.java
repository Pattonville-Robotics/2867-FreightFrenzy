package org.firstinspires.ftc.teamcode;

//USE THIS IF THE CAMERA STUFF DOESN'T WORK/
//ROBOT CHARGES STRAIGHT AHEAD LEEEEEEROYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY JENKINSSSSSSSSSS
//-Justin

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


@Autonomous(name="LastResort", group="Autonomous")
public class PickupMiddleBlock extends LinearOpMode {

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

        // Charge forward while spinning the hand
        waitForStart();
        arm.startIntake();
        encoder.moveInches(25, 0.7);
        wait(1000);
        arm.stopHand();

        // Move the arm up
        arm.moveToPosition(Arm.armPosition.TWO, 0.7);
        wait(2000);

        // Turn towards the shipping hub and move to it
        encoder.rotateDegrees(rotationalDirection.CLOCKWISE, 90, 0.7);
        encoder.moveInches(16, 0.5);

        // Spit out the block
        arm.startOuttake();
        wait(2500);
        arm.stopHand();

        //(Attempt to) move to the garage
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 4, 0.5);
        encoder.rotateDegrees(rotationalDirection.CLOCKWISE, 90, 0.7);
        encoder.moveInches(12, 0.7);
        encoder.rotateDegrees(rotationalDirection.COUNTERCLOCKWISE, 69, 0.7);
        encoder.moveInches(27, 0.7);
        encoder.rotateDegrees(rotationalDirection.COUNTERCLOCKWISE, 21, 0.5);
        encoder.moveInches(38, 0.7);



    }

}

