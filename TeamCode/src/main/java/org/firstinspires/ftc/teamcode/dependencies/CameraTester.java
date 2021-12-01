package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.opencv.core.Point;

@TeleOp
public class CameraTester extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        CameraColorSensor colorSensor = new CameraColorSensor ("Webcam", hardwareMap, this);
        
        // set up configuration parameters
        colorSensor.UsingWebcam = false;
        colorSensor.RegionTopLeft[0] = new Point(109, 98);
        colorSensor.RegionTopLeft[1] = new Point(181, 98);
        colorSensor.RegionTopLeft[2] = new Point(253, 98);
        colorSensor.RegionWidth = 60;
        colorSensor.RegionHeight = 60;
        
        waitForStart();
        
        while (opModeIsActive())
        {
            CameraColorSensor.Color_Enum color = CameraColorSensor.Color_Enum.Color_None;
            
            if (colorSensor.isRegionGreen(0)) color = CameraColorSensor.Color_Enum.Color_Green;
            else if (colorSensor.isRegionYellow(0)) color = CameraColorSensor.Color_Enum.Color_Yellow;
            else if (colorSensor.isRegionRed(0)) color = CameraColorSensor.Color_Enum.Color_Red;
            else if (colorSensor.isRegionBlue(0)) color = CameraColorSensor.Color_Enum.Color_Blue;
            else color = CameraColorSensor.Color_Enum.Color_None;
            
            telemetry.addData("Region 0", color);

            if (colorSensor.isRegionGreen(1)) color = CameraColorSensor.Color_Enum.Color_Green;
            else if (colorSensor.isRegionYellow(1)) color = CameraColorSensor.Color_Enum.Color_Yellow;
            else if (colorSensor.isRegionRed(1)) color = CameraColorSensor.Color_Enum.Color_Red;
            else if (colorSensor.isRegionBlue(1)) color = CameraColorSensor.Color_Enum.Color_Blue;
            else color = CameraColorSensor.Color_Enum.Color_None;
            
            telemetry.addData("Region 1", color);

            if (colorSensor.isRegionGreen(2)) color = CameraColorSensor.Color_Enum.Color_Green;
            else if (colorSensor.isRegionYellow(2)) color = CameraColorSensor.Color_Enum.Color_Yellow;
            else if (colorSensor.isRegionRed(2)) color = CameraColorSensor.Color_Enum.Color_Red;
            else if (colorSensor.isRegionBlue(2)) color = CameraColorSensor.Color_Enum.Color_Blue;
            else color = CameraColorSensor.Color_Enum.Color_None;
            
            telemetry.addData("Region 2", color);

            // print any telemetry data from the camera
            List<CameraColorSensor.TelemetryData> telemetryList = colorSensor.getTelemetryData();
            for (CameraColorSensor.TelemetryData data : telemetryList)
            {
                telemetry.addData(data.caption, data.object);
            }
            telemetry.update();
            sleep(100);
        }
    }
}
