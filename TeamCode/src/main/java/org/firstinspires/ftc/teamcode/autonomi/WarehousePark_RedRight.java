package org.firstinspires.ftc.teamcode.autonomi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.WarehousePark;
import org.firstinspires.ftc.teamcode.dependencies.AlliancePosition;
import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;

@Autonomous(name="WarehousePark_RedRight", group="Autonomous")
public class WarehousePark_RedRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WarehousePark.run(this, AllianceSide.RED, AlliancePosition.RIGHT);
    }
}

