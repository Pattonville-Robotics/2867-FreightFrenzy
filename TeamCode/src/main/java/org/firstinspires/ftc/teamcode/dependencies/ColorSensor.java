package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.dependencies.CameraColorSensor.Color_Enum;
import org.opencv.core.Point;

public class ColorSensor {
    CameraColorSensor camera;
    final Point[] DEFAULT_POINTS = {new Point(58, 110), new Point(130, 110), new Point(253, 110)};
    final int DEFAULT_WIDTH = 60;
    final int DEFAULT_HEIGHT = 60;
    public ColorSensor(String cameraName, HardwareMap hardwareMap, LinearOpMode linearOpMode){
        camera = new CameraColorSensor(cameraName, hardwareMap, linearOpMode);
        camera.UsingWebcam = false;
        camera.RegionTopLeft = DEFAULT_POINTS;
        camera.RegionWidth = DEFAULT_WIDTH;
        camera.RegionHeight = DEFAULT_HEIGHT;
    }
    public ColorSensor(String cameraName, HardwareMap hardwareMap, LinearOpMode linearOpMode, int width, int height, Point region1, Point region2, Point region3){
        camera = new CameraColorSensor(cameraName, hardwareMap, linearOpMode);
        camera.UsingWebcam = false;
        camera.RegionTopLeft[0] = region1;
        camera.RegionTopLeft[1] = region2;
        camera.RegionTopLeft[2] = region3;
        camera.RegionWidth = width;
        camera.RegionHeight = height;
    }
    public ColorSensor(String cameraName, HardwareMap hardwareMap, LinearOpMode linearOpMode, Point region1, Point region2, Point region3){
        camera = new CameraColorSensor(cameraName, hardwareMap, linearOpMode);
        camera.UsingWebcam = false;
        camera.RegionTopLeft[0] = region1;
        camera.RegionTopLeft[1] = region2;
        camera.RegionTopLeft[2] = region3;
        camera.RegionWidth = DEFAULT_WIDTH;
        camera.RegionHeight = DEFAULT_HEIGHT;
    }
    public ColorSensor(String cameraName, HardwareMap hardwareMap, LinearOpMode linearOpMode, int width, int height){
        camera = new CameraColorSensor(cameraName, hardwareMap, linearOpMode);
        camera.UsingWebcam = false;
        camera.RegionTopLeft = DEFAULT_POINTS;
        camera.RegionWidth = width;
        camera.RegionHeight = height;
    }
    public Color_Enum getColorAtRegion(int regionNumber){
        Color_Enum color;
        if (camera.isRegionGreen(regionNumber)) color = Color_Enum.Color_Green;
        else if (camera.isRegionYellow(regionNumber)) color = Color_Enum.Color_Yellow;
        else if (camera.isRegionRed(regionNumber)) color = Color_Enum.Color_Red;
        else if (camera.isRegionBlue(regionNumber)) color = Color_Enum.Color_Blue;
        else color = Color_Enum.Color_None;
        return color;
    }
    public boolean isRegionRed(int regionNumber){
        return camera.isRegionRed(regionNumber);
    }
    public boolean isRegionBlue(int regionNumber){
        return camera.isRegionBlue(regionNumber);
    }
    public boolean isRegionGreen(int regionNumber){
        return camera.isRegionRed(regionNumber);
    }
    public boolean isRegionYellow(int regionNumber){
        return camera.isRegionRed(regionNumber);
    }

}
