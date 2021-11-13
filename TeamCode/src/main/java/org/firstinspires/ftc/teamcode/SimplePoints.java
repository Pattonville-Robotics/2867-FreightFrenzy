package org.firstinspires.ftc.teamcode;

//Easy Points - written by Justin, we are literally just moving forwards

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;


@Autonomous(name="SimplePoints", group="Autonomous")
public class SimplePoints extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU imu;
    private TwoWheelEncoder encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare any local / helper variables here
        // Our initialization code should go here
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");


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
        telemetry.addData("Orientation: ", imu.getAngularOrientation());
        telemetry.update();
        //encoder.moveInches(Direction.FORWARD, 1, 0.5);
        encoder.rotateDegrees(rotationalDirection.CLOCKWISE, 270, 0.5);

    }

}

