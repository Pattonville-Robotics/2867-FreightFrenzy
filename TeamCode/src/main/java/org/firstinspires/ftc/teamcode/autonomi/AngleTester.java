package org.firstinspires.ftc.teamcode.autonomi;

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
import org.firstinspires.ftc.teamcode.dependencies.ScoopArm;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="AngleTester", group="Autonomous")
public class AngleTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare any local / helper variables here
        // Our initialization code should go here
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left");
        // private ColorSensor colorSensor;
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right");
         ScoopArm arm = new ScoopArm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));
        // colorSensor = new ColorSensor("Webcam", hardwareMap, this);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        TwoWheelEncoder encoder = new TwoWheelEncoder(leftDrive, rightDrive, imu, CommonParameters.FREIGHT_ROBOT, this);

        waitForStart();

        ArrayList<Double> errors = new ArrayList<>();
        ArrayList<Double> clockwiseErrors = new ArrayList<>();
        ArrayList<Double> counterclockwiseErrors = new ArrayList<>();

        // ඞ
        telemetry.speak("vine boom sound effect");

        arm.moveToPosition(Arm.armPosition.ONE, 0.3);

        // Run for clockwise and counterclockwise.
        for (int rotationI = 0; rotationI < 2; rotationI++){
            // Turn 10 times with different angles, measuring resulting angle and amount of offset
            rotationalDirection direction = rotationI==0 ? rotationalDirection.CLOCKWISE : rotationalDirection.COUNTERCLOCKWISE;
            for (int i=0; i<10; i++){
                double start = imu.getAngularOrientation().firstAngle;
                double angle = 15 + i*15;
                encoder.rotateDegrees(direction, angle);
                sleep(1069);
                double end = imu.getAngularOrientation().firstAngle;

                double error = rotationI==0 ? angle + (end-start) : angle - (end-start);
                telemetry.clearAll();
                telemetry.addData("Current angle: ", imu.getAngularOrientation().firstAngle);
                telemetry.addData("Direction: ", rotationI==0 ? "Clockwise" : "Counterclockwise");
                telemetry.addData("Expected: ", angle);
                telemetry.addData("Actual: ", (end-start));
                telemetry.addData("Error: ", error);
                telemetry.update();

                errors.add(error);
                if (rotationI == 0) {
                    clockwiseErrors.add(error);
                } else {
                    counterclockwiseErrors.add(error);
                }
            }
        }

        double total = 0; for (double d : errors){ total += d; }
        double clockwiseTotal = 0; for (double d : clockwiseErrors){ clockwiseTotal += d; }
        double counterclockwiseTotal = 0; for (double d : counterclockwiseErrors){ counterclockwiseTotal += d; }

        telemetry.clearAll();
        telemetry.addLine(clockwiseErrors.toString());
        telemetry.addData("Clockwise avg error: ", clockwiseTotal/10);
        telemetry.addLine(counterclockwiseErrors.toString());
        telemetry.addData("Counterclockwise avg error: ", counterclockwiseTotal/10);
        telemetry.addData("Combined avg error: ", total/20);
        telemetry.update();

        sleep(15000);
    }
}


