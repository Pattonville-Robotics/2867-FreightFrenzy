package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawArm extends Arm {
    private final Servo clawServo;
    private final static double OPEN_POSITION = 0;
    private final static double CLOSED_POSITION = 0.4;

    public ClawArm(DcMotor armMotor, Servo clawServo) {
        super(armMotor);
        this.clawServo = clawServo;
        clawServo.setPosition(OPEN_POSITION);
    }

    public void moveToPosition(armPosition pos, double power) {
        switch (pos) {
            case ONE: {
                moveToPosition(-100, power);
                break;
            }
            case TWO: {
                moveToPosition(-160, power);
                break;
            }
            case THREE: {
                moveToPosition(-236, power);
                break;
            }
            case NEUTRAL: {
                moveToPosition(-15, power);
                break;
            }
        }
    }

    public void open(){
        clawServo.setPosition(OPEN_POSITION);
    }
    public void close(){
        clawServo.setPosition(CLOSED_POSITION);
    }


    public void startIntake(){
        close();
    }

    public void startOuttake(){
        open();
    }

    public void stopHand(){};
}
