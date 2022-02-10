package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CarouselSpinner {
    public DcMotor spinner;
    private final static double SPIN_POWER = 0.22;

    public CarouselSpinner(DcMotor spinner){
        this.spinner=spinner;
    }
    public void spinRed(){
        spinner.setPower(SPIN_POWER);
    }
    public void spinBlue(){
        spinner.setPower(-SPIN_POWER);
    }
    public void stopSpin(){
        spinner.setPower(0);
    }
    public DcMotor getSpinner(){return spinner;}
}
