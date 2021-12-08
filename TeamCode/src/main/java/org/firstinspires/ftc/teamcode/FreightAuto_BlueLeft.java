package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dependencies.AlliancePosition;
import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;

@Autonomous(name="FreightAuto_BlueLeft", group="Autonomous")
public class FreightAuto_BlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FreightAuto.run(this, AllianceSide.BLUE, AlliancePosition.LEFT);
    }
}

