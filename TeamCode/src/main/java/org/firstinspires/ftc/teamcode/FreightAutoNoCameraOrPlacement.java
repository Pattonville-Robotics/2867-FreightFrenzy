package org.firstinspires.ftc.teamcode;

// This contains the same code as FreightAuto (12/7/21) but uses the saem autonomous structure as
// the other ones we've made without the wacky freightauto run() stuff.
// If the new freightAuto stuff doesn't work the

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.dependencies.AlliancePosition;
import org.firstinspires.ftc.teamcode.dependencies.AllianceSide;
import org.firstinspires.ftc.teamcode.dependencies.Arm;
import org.firstinspires.ftc.teamcode.dependencies.ColorSensor;
import org.firstinspires.ftc.teamcode.dependencies.CommonParameters;
import org.firstinspires.ftc.teamcode.dependencies.TwoWheelEncoder;
import org.firstinspires.ftc.teamcode.dependencies.rotationalDirection;

@Autonomous(name="FreightAutoNoCameraOrPlacement", group="Autonomous")
public class FreightAutoNoCameraOrPlacement extends LinearOpMode {
    private DcMotor leftDrive = null;
    private Arm arm;
    private ColorSensor colorSensor;
    private DcMotor rightDrive = null;
    private BNO055IMU imu;
    private TwoWheelEncoder encoder;

    private AllianceSide allianceSide = AllianceSide.RED;
    private AlliancePosition alliancePosition = AlliancePosition.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare any local / helper variables here
        // Our initialization code should go here
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        arm = new Arm(hardwareMap.get(DcMotor.class, "arm"), hardwareMap.get(CRServo.class, "scoop"));


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        encoder = new TwoWheelEncoder(leftDrive, rightDrive, imu, CommonParameters.FREIGHT_ROBOT, this);

        waitForStart();

        //Insert camera code here
        // for now, assume the duck is in the middle
        Arm.armPosition armPos = Arm.armPosition.TWO;

        // Move the arm to the appropriate height
        arm.moveToPosition(armPos, 0.7);
        encoder.moveInches(4);

        // Move to the hub (This will run while the arm is moving to save time)
        rotationalDirection towardsHub;
        rotationalDirection awayFromHub;
        if (alliancePosition == AlliancePosition.LEFT){
            towardsHub = rotationalDirection.CLOCKWISE;
            awayFromHub = rotationalDirection.COUNTERCLOCKWISE;
        } else {
            towardsHub = rotationalDirection.COUNTERCLOCKWISE;
            awayFromHub = rotationalDirection.CLOCKWISE;
        }
        encoder.rotateDegrees(towardsHub, 90);
        encoder.moveInches(20);
        encoder.rotateDegrees(awayFromHub, 90);
        encoder.moveInches(17);

        // Spit out the block
        arm.startOuttake();
        sleep(2000);
        arm.stopHand();

        // Move into the depot
        encoder.moveInches(DcMotorSimple.Direction.REVERSE, 17, 0.7);

        rotationalDirection towardsDepot;
        rotationalDirection awayFromDepot;
        if (allianceSide == AllianceSide.RED){
            towardsDepot = rotationalDirection.CLOCKWISE;
            awayFromDepot = rotationalDirection.COUNTERCLOCKWISE;
        } else {
            towardsDepot = rotationalDirection.COUNTERCLOCKWISE;
            awayFromDepot = rotationalDirection.CLOCKWISE;
        }
        encoder.rotateDegrees(towardsDepot, 90);
        encoder.moveInches(23);
        encoder.rotateDegrees(awayFromDepot, 6);
        encoder.moveInches(31);


    }

}

