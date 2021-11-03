package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotParameters {
    public static final int TICKS_CONSTANT = 28;

    private final double wheelBaseRadius, wheelRadius, driveGearRatio, gearBoxRatio;
    private final boolean gyroEnabled, encodersEnabled;
    private final DcMotorSimple.Direction leftDriveMotorDirection, rightDriveMotorDirection;

    /**
     * Cached computed values, never change since the class is final
     */
    private final double wheelCircumference, wheelBaseCircumference, adjustedTicksPerRevolution;

    private RobotParameters(double wheelBaseRadius, double wheelRadius, double driveGearRatio, double gearBoxRatio, boolean gyroEnabled, boolean encodersEnabled, DcMotorSimple.Direction leftDriveMotorDirection, DcMotorSimple.Direction rightDriveMotorDirection) {
        this.wheelBaseRadius = wheelBaseRadius;
        this.wheelRadius = wheelRadius;
        this.driveGearRatio = driveGearRatio;
        this.gyroEnabled = gyroEnabled;
        this.encodersEnabled = encodersEnabled;
        this.gearBoxRatio = gearBoxRatio;

        this.wheelCircumference = wheelRadius * 2 * Math.PI;
        this.wheelBaseCircumference = wheelBaseRadius * 2 * Math.PI;
        this.adjustedTicksPerRevolution = TICKS_CONSTANT * gearBoxRatio / driveGearRatio;
        this.leftDriveMotorDirection = leftDriveMotorDirection;
        this.rightDriveMotorDirection = rightDriveMotorDirection;
    }

    public double getWheelBaseRadius() {
        return wheelBaseRadius;
    }

    public double getWheelRadius() {
        return wheelRadius;
    }

    public double getAdjustedTicksPerRevolution() {
        return adjustedTicksPerRevolution;
    }

    public double getWheelCircumference() {
        return wheelCircumference;
    }

    public double getWheelBaseCircumference() {
        return wheelBaseCircumference;
    }

    public double getDriveGearRatio() {
        return driveGearRatio;
    }

    public double getGearBoxRatio() {
        return gearBoxRatio;
    }

    public boolean isGyroEnabled() {
        return gyroEnabled;
    }

    public DcMotorSimple.Direction getLeftDriveMotorDirection() {
        return leftDriveMotorDirection;
    }

    public DcMotorSimple.Direction getRightDriveMotorDirection() {
        return rightDriveMotorDirection;
    }

    public boolean areEncodersEnabled() {
        return encodersEnabled;
    }

    public static class Builder {
        private double wheelBaseRadius;
        private double wheelRadius;
        private double driveGearRatio = 1;
        private double gearBoxRatio = 60;
        private boolean gyroEnabled = false;
        private boolean encodersEnabled = false;
        private DcMotorSimple.Direction leftDriveMotorDirection = DcMotorSimple.Direction.FORWARD;
        private DcMotorSimple.Direction rightDriveMotorDirection = DcMotorSimple.Direction.REVERSE;

        public Builder() {
        }

        public org.firstinspires.ftc.teamcode.RobotParameters.Builder wheelBaseRadius(double wheelBaseRadius) {
            this.wheelBaseRadius = wheelBaseRadius;
            return this;
        }
        public org.firstinspires.ftc.teamcode.RobotParameters.Builder gearBoxRatio(double gearBoxRatio) {
            this.gearBoxRatio = gearBoxRatio;
            return this;
        }
        public org.firstinspires.ftc.teamcode.RobotParameters.Builder wheelRadius(double wheelRadius) {
            this.wheelRadius = wheelRadius;
            return this;
        }

        public org.firstinspires.ftc.teamcode.RobotParameters.Builder driveGearRatio(double driveGearRatio) {
            this.driveGearRatio = driveGearRatio;
            return this;
        }

        public org.firstinspires.ftc.teamcode.RobotParameters.Builder gyroEnabled(boolean gyroEnabled) {
            this.gyroEnabled = gyroEnabled;
            return this;
        }

        public org.firstinspires.ftc.teamcode.RobotParameters.Builder encodersEnabled(boolean encodersEnabled) {
            this.encodersEnabled = encodersEnabled;
            return this;
        }

        public org.firstinspires.ftc.teamcode.RobotParameters.Builder leftDriveMotorDirection(DcMotorSimple.Direction leftDriveMotorDirection) {
            this.leftDriveMotorDirection = leftDriveMotorDirection;
            return this;
        }

        public org.firstinspires.ftc.teamcode.RobotParameters.Builder rightDriveMotorDirection(DcMotorSimple.Direction rightDriveMotorDirection) {
            this.rightDriveMotorDirection = rightDriveMotorDirection;
            return this;
        }

        public org.firstinspires.ftc.teamcode.RobotParameters build() {
            if (wheelBaseRadius <= 0)
                throw new IllegalArgumentException("wheelBaseRadius must be > 0");
            if (wheelRadius <= 0)
                throw new IllegalArgumentException("wheelRadius must be > 0");
            return new org.firstinspires.ftc.teamcode.RobotParameters(wheelBaseRadius, wheelRadius, driveGearRatio, gearBoxRatio, gyroEnabled, encodersEnabled, leftDriveMotorDirection, rightDriveMotorDirection);
        }
    }
}