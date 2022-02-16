package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

// Claw with Wrist Arm - an arm that has a claw attached to it, of which a wrist is also attached
// which can bend the hand backward and allow it to drop items behind it when the arm is extended
// all the way backwards. Useful for delivering blocks backwards and for capping.

public class ClawWithWristArm extends Arm {
    private final CRServo claw;
    private final CRServo wrist;

    private final static double HAND_POWER = 1.0;
    private final static double WRIST_POWER = 0.7;

    public ClawWithWristArm(DcMotor armMotor, CRServo claw, CRServo wrist) {
        super(armMotor);
        this.claw = claw;
        this.wrist = wrist;
    }


    // == Hand
    public void closeHand(){
        this.claw.setPower(-HAND_POWER);
    }

    public void openHand(){
        this.claw.setPower(HAND_POWER);
    }

    public void stopHand(){
        this.claw.setPower(0);
    }

    public void setHandPower(double power){
        this.claw.setPower(power);
    }

    // Required implementations, needed for general Arm instances
    public void startIntake() { closeHand(); }

    public void startOuttake() { openHand(); }


    // === Wrist
    public void moveWristBack(){
        this.wrist.setPower(-WRIST_POWER);
    }

    public void moveWristForward(){
        this.wrist.setPower(WRIST_POWER);
    }

    public void stopWrist(){
        this.wrist.setPower(0);
    }

    public void setWristPower(double power){
        wrist.setPower(power);
    }


    // == Accessors
    public CRServo getClaw(){
        return this.claw;
    }

    public CRServo getWrist() {
        return wrist;
    }
}
