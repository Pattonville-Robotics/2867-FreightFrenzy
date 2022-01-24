package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawArm extends Arm {
    private final Servo clawServo;
    private final static double OPEN_POSITION = 0;
    private final static double CLOSED_POSITION = 0.5;

    public ClawArm(DcMotor armMotor, Servo clawServo) {
        super(armMotor);
        this.clawServo = clawServo;
        clawServo.setPosition(OPEN_POSITION);
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
