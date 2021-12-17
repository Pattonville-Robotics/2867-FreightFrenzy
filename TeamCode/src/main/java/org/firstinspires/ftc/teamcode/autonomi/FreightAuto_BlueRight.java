package org.firstinspires.ftc.teamcode.autonomi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FreightAuto;
import org.firstinspires.ftc.teamcode.dependencies.AlliancePosition;
import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;

@Autonomous(name="FreightAuto_BlueRight", group="Autonomous")
public class FreightAuto_BlueRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FreightAuto.run(this, AllianceSide.BLUE, AlliancePosition.RIGHT);
    }
}

