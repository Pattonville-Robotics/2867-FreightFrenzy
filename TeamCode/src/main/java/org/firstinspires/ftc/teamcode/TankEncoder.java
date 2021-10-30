package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class TankEncoder {


    public static final int TARGET_REACHED_THRESHOLD = 16;

    private static final String TAG = "TankEncoder";

    private DcMotor.RunMode leftDriveSavedMotorMode, rightDriveSavedMotorMode;

    private final RobotParameters ROBOTPARAMETERS;

    private final DcMotor leftDriveMotor, rightDriveMotor;

    private LinearOpMode linearOpMode;



    public TankEncoder(DcMotor leftDriveMotor, DcMotor rightDriveMotor, LinearOpMode linearOpMode){
        ROBOTPARAMETERS = new RobotParameters.Builder()
                .wheelRadius(2.5)
                .wheelBaseRadius(10)
                .build();

        this.leftDriveMotor = leftDriveMotor;
        this.rightDriveMotor = rightDriveMotor;
        this.linearOpMode = linearOpMode;
    }


    /**
     * NOT REVERSIBLE WITH <code>degreesToInches</code>!
     *
     * @param inches the number of inches to be covered by a single wheel
     * @return the number of encoder ticks to achieve that
     */
    public double inchesToTicks(double inches) {
        return ROBOTPARAMETERS.getAdjustedTicksPerRevolution() * inches / ROBOTPARAMETERS.getWheelCircumference();
    }
    public double inchesToTicksInverse(int ticks) {
        return ticks * ROBOTPARAMETERS.getWheelCircumference() / ROBOTPARAMETERS.getAdjustedTicksPerRevolution();
    }




    /**
     * NOT REVERSIBLE WITH <code>inchesToTicks</code>!
     *
     * @param degrees the number of degrees to turn the robot
     * @return the number of inches each wheel has to travel
     */
    public double degreesToInches(double degrees) {
        return ROBOTPARAMETERS.getWheelBaseCircumference() * degrees / 360;
    }

    public double degreesToInchesInverse(double inches) {
        return 360 * inches / ROBOTPARAMETERS.getWheelBaseCircumference();
    }


    public void moveFreely(double leftPower, double rightPower) {
        leftDriveMotor.setPower(leftPower);
        rightDriveMotor.setPower(rightPower);
    }

    public void move(DcMotorSimple.Direction direction, double power) {
        double motorPower;

        switch (direction) {
            case FORWARD:
                motorPower = power;
                break;
            case REVERSE:
                motorPower = -power;
                break;
            default:
                throw new IllegalArgumentException("Direction must be Reverse or Forwards");
        }

        moveFreely(motorPower, motorPower);
    }
    protected void setMotorsRunToPosition() {
        if (leftDriveMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (rightDriveMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void setMotorTargets(int targetPositionLeft, int targetPositionRight) {
        leftDriveMotor.setTargetPosition(targetPositionLeft);
        rightDriveMotor.setTargetPosition(targetPositionRight);
    }
    protected void restoreMotorModes() {
        leftDriveMotor.setMode(leftDriveSavedMotorMode);
        rightDriveMotor.setMode(rightDriveSavedMotorMode);
    }

    protected void storeMotorModes() {
        leftDriveSavedMotorMode = leftDriveMotor.getMode();
        rightDriveSavedMotorMode = rightDriveMotor.getMode();
    }

    protected void resetMotorEncoders() {
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void moveInches(DcMotorSimple.Direction direction, double inches, double power) {
        //Move Specified Inches Using Motor Encoders

        int targetPositionLeft;
        int targetPositionRight;

        //Log.e(TAG, "Getting motor modes");
        storeMotorModes();

        resetMotorEncoders();

        int deltaPosition = (int) Math.round(inchesToTicks(inches));

        switch (direction) {
            case FORWARD: {
                targetPositionLeft = deltaPosition;
                targetPositionRight = deltaPosition;
                break;
            }
            case REVERSE: {
                targetPositionLeft = -deltaPosition;
                targetPositionRight = -deltaPosition;
                break;
            }
            default:
                throw new IllegalArgumentException("Direction must be Direction.FORWARDS or Direction.BACKWARDS!");
        }

        //Log.e(TAG, "Setting motor modes");
        setMotorsRunToPosition();

        //Log.e(TAG, "Setting motor power high");
        move(DcMotorSimple.Direction.FORWARD, power); // To keep power in [0.0, 1.0]. Encoders control direction

        //Log.e(TAG, "Setting target position");
        setMotorTargets(targetPositionLeft, targetPositionRight);


        //int oldLeftPosition = leftDriveMotor.getCurrentPosition();
        //int oldRightPosition = rightDriveMotor.getCurrentPosition();

        while ((leftDriveMotor.isBusy() || rightDriveMotor.isBusy()) || !reachedTarget(leftDriveMotor.getCurrentPosition(), targetPositionLeft, rightDriveMotor.getCurrentPosition(), targetPositionRight) && !linearOpMode.isStopRequested() && linearOpMode.opModeIsActive()) {
            Thread.yield();
            //distance.setValue("DistanceL: " + leftDriveMotor.getCurrentPosition() + " DistanceR: " + rightDriveMotor.getCurrentPosition());
            linearOpMode.telemetry.update();
        }
        //Log.e(TAG, "Setting motor power low");
        stop();

        //Log.e(TAG, "Restoring motor mode");
        restoreMotorModes();

        sleep(100);
    }
    public void stop() {
        moveFreely(0, 0);
    }

    public void sleep(long milli) {
        this.linearOpMode.sleep(milli);
    }



    protected boolean reachedTarget(int currentPositionLeft, int targetPositionLeft, int currentPositionRight, int targetPositionRight) {
        return Math.abs(currentPositionLeft - targetPositionLeft) < TARGET_REACHED_THRESHOLD && Math.abs(currentPositionRight - targetPositionRight) < TARGET_REACHED_THRESHOLD;
    }
}
