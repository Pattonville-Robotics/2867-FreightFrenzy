package org.firstinspires.ftc.teamcode;

//AutoCameraPoints - Justin's horrible attempt at camera detection in autonomous

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dependencies.Arm;
import org.firstinspires.ftc.teamcode.dependencies.ColorSensor;
import org.firstinspires.ftc.teamcode.dependencies.CommonParameters;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;


@Autonomous(name="AutoCameraPoints", group="Autonomous")
public class AutoCameraPoints extends LinearOpMode {

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
        //code go here
        //encoder.moveInches(Direction.FORWARD, 10, 0.5);
        if(colorSensor.isRegionYellow(0)){ //square left
            //insert code to go to thing and pick it up
            arm.moveToPosition(Arm.armPosition.ONE, 0.7);
            //insert code to release
        }
        else if(colorSensor.isRegionYellow(1)) { //square center
            //insert code to go to thing and pick it up
            arm.moveToPosition(Arm.armPosition.TWO, 0.7);
            //insert code to release
        }
        else{ //this means it must be square right
            //insert code to go to thing and pick it up
            arm.moveToPosition(Arm.armPosition.THREE, 0.7);
            //insert code to release
        }


//        while(this.opModeIsActive()){
//            telemetry.clearAll();
//            telemetry.addData("Red: ", colorSensor.isRedPresent());
//            telemetry.update();
//        }

    }

}

