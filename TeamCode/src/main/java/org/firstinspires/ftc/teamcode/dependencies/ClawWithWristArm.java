package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawWithWristArm extends Arm {
    private final CRServo handServo;
    private final CRServo wrist;
    private final static double SERVO_POWER = 1.0;

    public ClawWithWristArm(DcMotor armMotor, CRServo handServo, CRServo wrist) {
        super(armMotor);
        this.handServo = handServo;
        this.wrist = wrist;
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

    public void setClawPower(double power){
        this.handServo.setPower(power);
    }

    public void setWristPower(double power){
        wrist.setPower(power);
    }

    public CRServo getHandServo(){
        return this.handServo;
    }

    public CRServo getWrist() {
        return wrist;
    }
}
