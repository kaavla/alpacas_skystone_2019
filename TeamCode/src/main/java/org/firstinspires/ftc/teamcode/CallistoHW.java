package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class CallistoHW
{
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor backrightMotor = null;
    public DcMotor backleftMotor = null;

    public DcMotor MCollectionSlide = null;
    public DcMotor MCollectionLift = null;
    public DcMotor MDropLift = null;
    public DcMotor MLanderLift = null;

    public CRServo spinnerServo = null;
    public Servo trayServo = null;
    public Servo markerServo = null;

    public DigitalChannel digitalTouch = null;  // Hardware Device Object
    //public DistanceSensor sensorRange = null;


    //static final double     REFERENCE_ANGLE           = 165;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    public BNO055IMU imu = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        RobotLog.ii("CAL", "Enter - init");

        leftMotor = ahwMap.get(DcMotor.class, "MFrontLeft");
        rightMotor = ahwMap.get(DcMotor.class, "MFrontRight");
        backleftMotor = ahwMap.get(DcMotor.class, "MBackLeft");
        backrightMotor = ahwMap.get(DcMotor.class, "MBackRight");

        MCollectionSlide = ahwMap.get(DcMotor.class, "MCollectionSlide");
        MCollectionLift = ahwMap.get(DcMotor.class, "MCollectionLift");
        MDropLift = ahwMap.get(DcMotor.class, "MDropLift");
        MLanderLift = ahwMap.get(DcMotor.class, "MLanderLift");

        spinnerServo = ahwMap.get(CRServo.class, "spinnerServo");
        trayServo = ahwMap.get(Servo.class, "trayServo");
        markerServo = ahwMap.get(Servo.class, "markerServo");

        digitalTouch = ahwMap.get(DigitalChannel.class, "sensor_digital");
        //sensorRange = ahwMap.get(DistanceSensor.class, "sensor_range");



        imu = ahwMap.get(BNO055IMU.class, "imu 1");

        //initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        //Invert direction for left motors
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        MLanderLift.setDirection(DcMotorSimple.Direction.FORWARD);
        // Set all motors to zero power
        stopAllMotors();

        //Set zero power behavior to braking
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotLog.ii("CAL", "Exit - init");

    }

    public void stopAllMotors()
    {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backleftMotor.setPower(0);
        backrightMotor.setPower(0);

        MCollectionSlide.setPower(0);
        MCollectionLift.setPower(0);
        MDropLift.setPower(0);
        MLanderLift.setPower(0);

        spinnerServo.setPower(0);
    }

    public void moveForward(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(power);
    }

    public void moveBackwards(double power)
    {
        leftMotor.setPower(-1 * power);
        rightMotor.setPower(-1 * power);
        backleftMotor.setPower(-1 * power);
        backrightMotor.setPower(-1 * power);
    }

    public void turnRight(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(-1 * power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(-1 * power);
    }

    public void turnLeft(double power)
    {
        leftMotor.setPower(-1 * power);
        rightMotor.setPower(power);
        backleftMotor.setPower(-1 * power);
        backrightMotor.setPower(power);
    }

    public void strafeRight(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(-1 * power);
        backleftMotor.setPower(-1 * power);
        backrightMotor.setPower(power);
    }

    public void strafeleft(double power)
    {
        leftMotor.setPower(-1 * power);
        rightMotor.setPower(power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(-1 * power);
    }

    public void diagonalforwardRight(double power)
    {
        leftMotor.setPower(power);
        backrightMotor.setPower(power);
    }

    public void diagonalforwardLeft(double power)
    {
        rightMotor.setPower(power);
        backleftMotor.setPower(power);
    }

    public void diagonalbackwardsRight(double power)
    {
        rightMotor.setPower(-1 * power);
        backleftMotor.setPower(-1 * power);
    }

    public void diagonalbackwardsLeft(double power)
    {
        leftMotor.setPower(-1 * power);
        backrightMotor.setPower(-1 * power);
    }

    public void forwardSlow()
    {
        leftMotor.setPower(Range.clip(leftMotor.getPower() + 0.01, 0.3, 1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() + 0.01, 0.3, 1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() + 0.01, 0.3, 1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() + 0.01, 0.3, 1.0));

    }

    public void backwardSlow()
    {
        leftMotor.setPower(Range.clip(leftMotor.getPower() - 0.01, -0.3, -1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() - 0.01, -0.3, -1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() - 0.01, -0.3, -1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() - 0.01, -0.3, -1.0));
    }


    public void collectionSlideOut(double power)
    {
        MCollectionSlide.setPower(-1*power);
    }

    public void collectionSlideIn(double power)
    {
        MCollectionSlide.setPower(power);
    }

    public void collectionLiftUp(double power)
    {
        MCollectionLift.setPower(-1*power);
    }

    public void collectionLiftDown(double power)
    {
        MCollectionLift.setPower(power);
    }

    public void collectionDropLiftUp(double power)
    {
        MDropLift.setPower(-1*power);
    }

    public void collectionDropLiftDown(double power)
    {
        MDropLift.setPower(power);
    }
    public void landerliftUp(double power)
     {

        MLanderLift.setPower(power);
    }

    public void landerliftDown(double power)
    {
        MLanderLift.setPower(-1*power);
    }

    public void turnspinnerservoforward(double power)
    {
        spinnerServo.setPower(power);
    }

    public void turnspinnerservobacwards(double power)
    {
        spinnerServo.setPower(-1 * power);
    }

    public void turnTraytocollect()
    {
        trayServo.setPosition(0);
    }

    public void turnTraytodrop()
    {
        trayServo.setPosition(0.6);
    }

    public void turnMarkerServotoInitPos()
    {
        markerServo.setPosition(0);
    }
    public void turnMarkerServotoDrop()
    {
        markerServo.setPosition(0.7);
    }


}
