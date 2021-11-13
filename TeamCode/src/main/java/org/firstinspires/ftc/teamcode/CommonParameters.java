package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CommonParameters {
    public static final RobotParameters TEST_ROBOT = new RobotParameters.Builder()
            .wheelRadius(2.55)
            .wheelBaseRadius(5)
            .gearBoxRatio(60)
            .rightDriveMotorDirection(DcMotorSimple.Direction.REVERSE)
            .build();
    public static final RobotParameters FREIGHT_ROBOT = new RobotParameters.Builder()
            .wheelRadius(1.7)
            .wheelBaseRadius(5.5)
            .gearBoxRatio(40)
            .leftDriveMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightDriveMotorDirection(DcMotorSimple.Direction.FORWARD)
            .build();
}
