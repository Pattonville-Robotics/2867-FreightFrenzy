package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.DcMotor;



public class Arm {
    private final DcMotor armMotor, handMotor;
    public enum armPosition{
        ONE, TWO, THREE, NEUTRAL
    }

    public Arm(DcMotor armMotor, DcMotor handMotor) {
        this.armMotor = armMotor;
        this.handMotor = handMotor;
        if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    /*
    public void moveToPosition(armPosition pos, double power){
        switch (pos){
            case ONE:{
                //do stuff 1
                break;
            }
            case TWO:{
                //do stuff 2
                break;
            }
            case THREE:{
                //do stuff 3
                break;
            }
            case NEUTRAL:{
                //do stuff ඞ
                break;
            }
        }
    }
    */
    public void moveToPosition(int pos, double power){
        if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armMotor.setTargetPosition(pos);
        if(armMotor.getCurrentPosition()<pos) {
            armMotor.setPower(power);
        }else if(armMotor.getCurrentPosition()>pos){
            armMotor.setPower(-power);
        }
        while(armMotor.isBusy()&&(Math.abs(armMotor.getTargetPosition()-armMotor.getCurrentPosition())>10)){
            Thread.yield();
        }
        armMotor.setPower(0);
    }
    public void startIntake(){
        this.handMotor.setPower(0.2);
    }
    public void startOuttake(){
        this.handMotor.setPower(-0.2);
    }
    public void stop(){
        this.handMotor.setPower(0);
    }

    public DcMotor getArmMotor() {
        return this.armMotor;
    }
    public DcMotor getHandMotor(){
        return this.handMotor;
    }
}
