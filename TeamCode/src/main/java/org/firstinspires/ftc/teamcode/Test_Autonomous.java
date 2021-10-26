package org.firstinspires.ftc.teamcode;

//Test autonomous - written by jack, testing out how to run autonomous code

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Autonomous Test", group="Autonomous")
public class Test_Autonomous extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare any local / helper variables here

        // Our initialization code should go here
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        long start = System.currentTimeMillis();
        for (int i = 0; i<4; i++) {
            leftDrive.setPower(-0.5);
            rightDrive.setPower(0.5);

            sleep(3000);

            leftDrive.setPower(.7);
            rightDrive.setPower(.7);

            sleep(3300);
        }
    }
}
