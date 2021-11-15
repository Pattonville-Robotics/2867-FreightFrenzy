package org.firstinspires.ftc.teamcode.dependencies;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.dependencies.RobotParameters;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;
import org.firstinspires.ftc.teamcode.dependencies.rotationalMeasure;


public class TwoWheelEncoder {


    public static final int TARGET_REACHED_THRESHOLD = 10;
    public static final int DEGREE_OF_ERROR = 50;

    private static final String TAG = "TwoWheelEncoder";

    private DcMotor.RunMode leftDriveSavedMotorMode, rightDriveSavedMotorMode;

    private final RobotParameters ROBOTPARAMETERS;
    private final BNO055IMU imu;
    private final DcMotor leftDriveMotor, rightDriveMotor;

    private final LinearOpMode linearOpMode;


    public TwoWheelEncoder(DcMotor leftDriveMotor, DcMotor rightDriveMotor, BNO055IMU imu, RobotParameters robotParameters, LinearOpMode linearOpMode){
        this.ROBOTPARAMETERS = robotParameters;
        this.leftDriveMotor = leftDriveMotor;
        this.rightDriveMotor = rightDriveMotor;
        this.linearOpMode = linearOpMode;
        this.imu = imu;
    }


    /**
     * NOT REVERSIBLE WITH <code>degreesToInches</code>!
     *
     * @param inches the number of inches to be covered by a single wheel
     * @return the number of encoder ticks to achieve that
     */
    public double inchesToTicks(double inches) {
        return this.ROBOTPARAMETERS.getAdjustedTicksPerRevolution() * inches / this.ROBOTPARAMETERS.getWheelCircumference();
    }
    public double inchesToTicksInverse(int ticks) {
        return ticks * this.ROBOTPARAMETERS.getWheelCircumference() / this.ROBOTPARAMETERS.getAdjustedTicksPerRevolution();
    }
    public rotationalDirection getRotationalInverse(rotationalDirection start){
        switch (start) {
            case CLOCKWISE:
                return rotationalDirection.COUNTERCLOCKWISE;
            case COUNTERCLOCKWISE:
                return rotationalDirection.CLOCKWISE;
            default:
                return rotationalDirection.CLOCKWISE;
        }
    }



    /**
     * NOT REVERSIBLE WITH <code>inchesToTicks</code>!
     *
     * @param degrees the number of degrees to turn the robot
     * @return the number of inches each wheel has to travel
     */
    public double degreesToInches(double degrees) {
        return this.ROBOTPARAMETERS.getWheelBaseCircumference() * degrees / 360;
    }

    public double degreesToInchesInverse(double inches) {
        return 360 * inches / this.ROBOTPARAMETERS.getWheelBaseCircumference();
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
    /**
    * Rotates the robot a set amount of degrees in the specified direction.
     * Only functional for powers <= 0.5.
    *
    * @param direction the direction to turn in. Uses rotationalDirection instead of DcSimpleMotor.Direction.
    * @param degrees the number of degrees to turn the robot.
     * @param power the power to turn the wheels at. Keep it below 0.6.
     */
    public void rotateDegrees(rotationalDirection direction, double degrees, double power){
        double startAngle = imu.getAngularOrientation().firstAngle;
        rotationalMeasure intendedAngle;

        if(direction == rotationalDirection.CLOCKWISE) {
            intendedAngle = new rotationalMeasure(startAngle - degrees);
            moveFreely(power, power);
        }else{
            intendedAngle = new rotationalMeasure(startAngle + degrees);
            moveFreely(-power, -power);
        }

        while(
                !(Math.abs(imu.getAngularOrientation().firstAngle-intendedAngle.get())<(DEGREE_OF_ERROR*power))&&
                (linearOpMode.opModeIsActive())
        ){
            Thread.yield();
            linearOpMode.telemetry.update();
        }
        stop();
        sleep(100);
    }
    public boolean motorsBusy(){
        return (leftDriveMotor.isBusy() || rightDriveMotor.isBusy());
    }
    public int[] getTargetPositions(DcMotorSimple.Direction direction, int ticks){
        int targetPositionLeft;
        int targetPositionRight;
        switch (direction) {
            case FORWARD: {
                if(ROBOTPARAMETERS.getLeftDriveMotorDirection() == DcMotorSimple.Direction.FORWARD) {
                    targetPositionLeft = -ticks;
                }else{
                    targetPositionLeft = ticks;
                }
                if(ROBOTPARAMETERS.getRightDriveMotorDirection() == DcMotorSimple.Direction.REVERSE) {
                    targetPositionRight = ticks;
                }else{
                    targetPositionRight = -ticks;
                }
                break;
            }
            case REVERSE: {
                if(ROBOTPARAMETERS.getLeftDriveMotorDirection() == DcMotorSimple.Direction.FORWARD) {
                    targetPositionLeft = ticks;
                }else{
                    targetPositionLeft = -ticks;
                }
                if(ROBOTPARAMETERS.getRightDriveMotorDirection() == DcMotorSimple.Direction.REVERSE) {
                    targetPositionRight = -ticks;
                }else{
                    targetPositionRight = ticks;
                }
                break;
            }
            default: {
                throw new IllegalArgumentException("Direction must be Direction.FORWARDS or Direction.BACKWARDS!");
            }
        }
        int[] result = new int[2];
        result[0] = targetPositionLeft;
        result[1] = targetPositionRight;
        return result;
    }
    public void moveInches(DcMotorSimple.Direction direction, double inches, double power) {
        //Move Specified Inches Using Motor Encoders


        //Log.e(TAG, "Getting motor modes");
        storeMotorModes();

        resetMotorEncoders();

        int deltaPosition = (int) Math.round(inchesToTicks(inches));
        int[] positions = getTargetPositions(direction, deltaPosition);


        //Log.e(TAG, "Setting motor modes");
        setMotorTargets(positions[0], positions[1]);

        setMotorsRunToPosition();

        //Log.e(TAG, "Setting motor power high");
        move(DcMotorSimple.Direction.FORWARD, power); // To keep power in [0.0, 1.0]. Encoders control direction

        //Log.e(TAG, "Setting target position");


        //int oldLeftPosition = leftDriveMotor.getCurrentPosition();
        //int oldRightPosition = rightDriveMotor.getCurrentPosition();

        while ((leftDriveMotor.isBusy() || rightDriveMotor.isBusy()) || !reachedTarget(leftDriveMotor.getCurrentPosition(), positions[0], rightDriveMotor.getCurrentPosition(), positions[1]) && !linearOpMode.isStopRequested() && linearOpMode.opModeIsActive()) {
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
