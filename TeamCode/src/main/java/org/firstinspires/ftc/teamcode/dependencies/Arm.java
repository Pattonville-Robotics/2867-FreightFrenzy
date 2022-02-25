package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.robotcore.hardware.DcMotor;

// Abstract class for an arm that has a hand attached to it.
// As of right now the only extension of this class that is really used is ClawWithWristArm
// but that may change in the future.

public abstract class Arm {
    private final DcMotor armMotor;

    public enum ArmPosition {
        NEUTRAL(0), ONE(70), TWO(133), THREE(233),
        BACK_NEUTRAL(590), BACK_ONE(555), BACK_TWO(530), BACK_THREE(487),
        CAP(445);

        public final int ticks;
        ArmPosition(int i) {
            this.ticks = i;
        }
    }

    public Arm(DcMotor armMotor) {
        this.armMotor = armMotor;
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void moveToPosition(ArmPosition pos, double power){
        moveToPosition(pos.ticks, power);
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
    public void stopArm(){
        this.armMotor.setPower(0);
    }

    public DcMotor getArmMotor() {
        return this.armMotor;
    }

    // MUST BE IMPLEMENTED IN CHILD CLASSES, AN ARM MUST HAVE A HAND THAT CAN
    // INTAKE, OUTTAKE, AND STOP INTAKING / OUTTAKING (literally 1984)
    public abstract void startIntake();

    public abstract void startOuttake();

    public abstract void stopHand();
}
