package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.dependencies.AlliancePosition;
import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;

@Autonomous(name="FreightAuto_RedRight", group="Autonomous")
public class FreightAuto_RedRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FreightAuto.run(this, AllianceSide.RED, AlliancePosition.RIGHT);
    }
}

