package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class TwoWheelEncoder {


    public static final int TARGET_REACHED_THRESHOLD = 10;
    public static final int DEGREE_OF_ERROR = 5;

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

    public void rotateDegrees(rotationalDirection direction, double degrees, double power){
        double startAngle = imu.getAngularOrientation().firstAngle;

        double intendedAngle;

        if(direction == rotationalDirection.CLOCKWISE) {
            intendedAngle = startAngle - degrees;
            moveFreely(power, power);
        }else{
            intendedAngle = startAngle + degrees;
            moveFreely(-power, -power);
        }

        while(
                !(Math.abs(imu.getAngularOrientation().firstAngle-intendedAngle)<DEGREE_OF_ERROR)&&
                (linearOpMode.opModeIsActive())
        ){
            Thread.yield();
        }
        stop();
        restoreMotorModes();
        sleep(100);
    }
    public boolean motorsBusy(){
        return (leftDriveMotor.isBusy() || rightDriveMotor.isBusy());
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
                targetPositionLeft = -deltaPosition;
                targetPositionRight = deltaPosition;
                break;
            }
            case REVERSE: {
                targetPositionLeft = deltaPosition;
                targetPositionRight = -deltaPosition;
                break;
            }
            default:
                throw new IllegalArgumentException("Direction must be Direction.FORWARDS or Direction.BACKWARDS!");
        }

        //Log.e(TAG, "Setting motor modes");
        setMotorTargets(targetPositionLeft, targetPositionRight);

        setMotorsRunToPosition();

        //Log.e(TAG, "Setting motor power high");
        move(DcMotorSimple.Direction.FORWARD, power); // To keep power in [0.0, 1.0]. Encoders control direction

        //Log.e(TAG, "Setting target position");


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
