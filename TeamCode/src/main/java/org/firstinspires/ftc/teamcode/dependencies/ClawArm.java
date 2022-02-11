package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawArm extends Arm {
    private final CRServo clawServo;
    private final static double POWER = 0.7;

    public ClawArm(DcMotor armMotor, CRServo clawServo) {
        super(armMotor);
        this.clawServo = clawServo;
    }
    @Override
    public void moveToPosition(armPosition pos, double power) {
        switch (pos) {
            case ONE: {
                moveToPosition(100, power);
                break;
            }
            case TWO: {
                moveToPosition(160, power);
                break;
            }
            case THREE: {
                moveToPosition(236, power);
                break;
            }
            case NEUTRAL: {
                // claw rests slightly above the ground because the claw does not need to be directly on the
                // ground to pick up the block like the scoop did, and it prevents the claw from slamming
                // into the ground
                moveToPosition(0, power);
                break;
            }
        }
    }

    public void startIntake(){
        clawServo.setPower(POWER);
    }

    public void startOuttake(){
        clawServo.setPower(-POWER);
    }

    public void stopHand(){};
}
