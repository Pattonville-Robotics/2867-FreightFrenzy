package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Arm {
    private final DcMotor armMotor;
    private final CRServo handServo;
    private final static double SERVO_POWER = 1.0;
    public enum armPosition{
        ONE, TWO, THREE, NEUTRAL
    }

    public Arm(DcMotor armMotor, CRServo handServo) {
        this.armMotor = armMotor;
        this.handServo = handServo;
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void moveToPosition(armPosition pos, double power){
        switch (pos){
            case ONE:{
                moveToPosition(-100, power);
                break;
            }
            case TWO:{
                moveToPosition(-140, power);
                break;
            }
            case THREE:{
                moveToPosition(-227, power);
                break;
            }
            case NEUTRAL:{
                moveToPosition(0, power);
                break;
            }
        }
    }

    public void moveToPosition(int ticks, double power){
        if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armMotor.setTargetPosition(ticks);
        armMotor.setPower(power);
        /*
        while(armMotor.isBusy()&&(Math.abs(armMotor.getTargetPosition()-armMotor.getCurrentPosition())>10)){
            Thread.yield();
        }
        armMotor.setPower(0);

         */
    }
    public void setArmPower(double power){
        this.armMotor.setPower(power);
    }
    public void startIntake(){
        this.handServo.setPower(-SERVO_POWER);
    }
    public void startOuttake(){
        this.handServo.setPower(SERVO_POWER);
    }
    public void stopHand(){
        this.handServo.setPower(0);
    }
    public void stopArm(){
        this.armMotor.setPower(0);
    }

    public DcMotor getArmMotor() {
        return this.armMotor;
    }
    public CRServo getHandServo(){
        return this.handServo;
    }
}
