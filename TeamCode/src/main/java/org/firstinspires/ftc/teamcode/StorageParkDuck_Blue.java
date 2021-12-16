package org.firstinspires.ftc.teamcode;

// Drives forward and drops the block, thats it.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;


@Autonomous(name="StorageParkDuck_Blue", group="Autonomous")
public class StorageParkDuck_Blue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        StorageParkDuck.run(this, AllianceSide.BLUE);
    }
}

