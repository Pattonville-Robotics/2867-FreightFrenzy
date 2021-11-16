package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Arm {
    private final DcMotor armMotor;
    private final Servo handServo;
    private final static double MAX_SERVO_POSITION = Servo.MAX_POSITION;
    public enum armPosition{
        ONE, TWO, THREE, NEUTRAL
    }

    public Arm(DcMotor armMotor, Servo handServo) {
        this.armMotor = armMotor;
        this.handServo = handServo;
        if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        handServo.setPosition(MAX_SERVO_POSITION);
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
        //armMotor.setPower(0);
    }

    public void startIntake(){
        this.handServo.setPosition(0);
    }
    public void startOuttake(){
        this.handServo.setPosition(MAX_SERVO_POSITION);
    }
    public void stopArm(){
        this.armMotor.setPower(0);
    }

    public DcMotor getArmMotor() {
        return this.armMotor;
    }
    public Servo getHandServo(){
        return this.handServo;
    }
}
