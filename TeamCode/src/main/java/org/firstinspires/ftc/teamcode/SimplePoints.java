package org.firstinspires.ftc.teamcode;

//Easy Points - written by Justin, we are literally just moving forwards

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;

@Autonomous(name="SimplePoints", group="Autonomous")
public class SimplePoints extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU imu;
    private TankEncoder encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare any local / helper variables here
        // Our initialization code should go here
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        encoder = new TankEncoder(leftDrive, rightDrive, this);

//        // IMU, used for orientation
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu.initialize(parameters);
        waitForStart();
        encoder.moveInches(DcMotorSimple.Direction.FORWARD, 100, 1);
        /*
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        telemetry.addData("Orientation: ", imu.getAngularOrientation().firstAngle);
//        telemetry.update();

        long start = System.currentTimeMillis();
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);

        //leftDrive.

        sleep(3000);
        leftDrive.setPower(.7);
        rightDrive.setPower(.7);

         */
    }
}

