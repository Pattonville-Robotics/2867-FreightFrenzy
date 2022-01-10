package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CarouselSpinner {
    public DcMotor leftSpinner, rightSpinner;
    private final static double RIGHT_POWER = 0.22;
    private final static double LEFT_POWER = 1.0;

    public CarouselSpinner(DcMotor leftSpinner, DcMotor rightSpinner){
        this.leftSpinner=leftSpinner;
        this.rightSpinner=rightSpinner;
    }
    public void startSpin(){
        leftSpinner.setPower(LEFT_POWER);
        rightSpinner.setPower(RIGHT_POWER);
    }
    public void stopSpin(){
        leftSpinner.setPower(0);
        rightSpinner.setPower(0);
    }
    public DcMotor getLeft(){
        return leftSpinner;
    }
    public DcMotor getRight(){
        return rightSpinner;
    }
}
