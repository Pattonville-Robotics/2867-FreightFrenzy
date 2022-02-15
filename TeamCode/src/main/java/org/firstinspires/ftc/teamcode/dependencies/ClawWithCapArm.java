package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ClawWithCapArm extends Arm {
    private final CRServo handServo;
    private final static double SERVO_POWER = 1.0;

    public ClawWithCapArm(DcMotor armMotor, CRServo handServo) {
        super(armMotor);
        this.handServo = handServo;
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

    public void setScoopPower(double power){
        this.handServo.setPower(power);
    }

    public CRServo getHandServo(){
        return this.handServo;
    }
}
