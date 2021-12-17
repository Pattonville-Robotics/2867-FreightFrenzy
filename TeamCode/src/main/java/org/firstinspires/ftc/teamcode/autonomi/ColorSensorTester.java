package org.firstinspires.ftc.teamcode.autonomi;

//Tests the camera functions

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


@Autonomous
public class ColorSensorTester extends LinearOpMode {

    //private ColorSensorOld colorSensor;
    private ColorSensor colorSensor;

    //@Override
    public void runOpMode() throws InterruptedException {

        // Declare any local / helper variables here
        // Our initialization code should go here
        //colorSensor = new ColorSensorOld("Webcam", hardwareMap, this);
        colorSensor = new ColorSensor("Webcam", hardwareMap, this);


        //colorSensor = new ColorSensor("Webcam", hardwareMap, this);

        waitForStart();
        while(opModeIsActive()){
            telemetry.clearAll();
            try {
                telemetry.addData("Reegion Left: ", colorSensor.getColorAtRegion(0));
                telemetry.addData("Region Middle: ", colorSensor.getColorAtRegion(1));
                telemetry.addData("Region Right: ", colorSensor.getColorAtRegion(2));
            }catch(NullPointerException e){
                telemetry.addData("Reegion Left: ", "Camera Inactive");
                telemetry.addData("Region Middle: ", "Camera Inactive");
                telemetry.addData("Region Right: ", "Camera Inactive");
            }
            //telemetry.addData("Region Left: ", colorSensor.isRedPresent());
            telemetry.update();
        }


    }

}

