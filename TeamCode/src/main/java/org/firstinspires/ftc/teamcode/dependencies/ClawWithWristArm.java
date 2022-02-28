package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// Claw with Wrist Arm - an arm that has a claw attached to it, of which a wrist is also attached
// which can bend the hand backward and allow it to drop items behind it when the arm is extended
// all the way backwards. Useful for delivering blocks backwards and for capping.

public class ClawWithWristArm extends Arm {
    private final CRServo claw;
    private final Servo wrist;

    private final static double HAND_POWER = 1.0;

    public ClawWithWristArm(DcMotor armMotor, CRServo claw, Servo wrist) {
        super(armMotor);
        this.claw = claw;
        this.wrist = wrist;
    }


    // == Hand
    public void closeHand(){
        claw.setPower(HAND_POWER);
    }

    public void openHand(){
        claw.setPower(-HAND_POWER);
    }

    public void stopHand(){
        claw.setPower(0);
    }

    public void setHandPower(double power){
        this.claw.setPower(power);
    }

    // Required implementations, needed for general Arm instances
    public void startIntake() { closeHand(); }

    public void startOuttake() { openHand(); }


    // === Wrist
    public void wristDown(){
        if((ArmPosition.CAP.ticks>this.currentPosition)&&(this.currentPosition>=ArmPosition.TWO.ticks)){
            wrist.setPosition(1);
        }else {
            wrist.setPosition(0.7);
        }
    }
    public void wristCap(){
        wrist.setPosition(0.35);
    }
    public void wristUp(){
        wrist.setPosition(0.0);
    }


    // == Accessors
    public CRServo getClaw(){
        return this.claw;
    }

    public Servo getWrist() {
        return wrist;
    }
}
