package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class JARVISHW
{
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor backrightMotor = null;
    public DcMotor backleftMotor = null;

    public DcMotor slide_1 = null;
    public DcMotor slide_2 = null;
    public DcMotor slide_3 = null;
    public CRServo turnServo = null;
    public Servo clawServo = null;


    public Servo FLServo = null;
    public Servo FRServo = null;

    //public ColorSensor sensorColor = null;

    Orientation lastAngles = new Orientation();  //?
    double globalAngle, power = .30, correction;  //?
    //sets the power used in each of the actions

    public BNO055IMU imu = null;



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        RobotLog.ii("CAL", "Enter - init");

        leftMotor = ahwMap.get(DcMotor.class, "M1");
        rightMotor = ahwMap.get(DcMotor.class, "M2");
        backleftMotor = ahwMap.get(DcMotor.class, "M3");
        backrightMotor = ahwMap.get(DcMotor.class, "M4");

        slide_1  = ahwMap.get(DcMotor.class, "slide_1");
        slide_2  = ahwMap.get(DcMotor.class, "slide_2");
        slide_3 = ahwMap.get(DcMotor.class, "slide_3");

        turnServo = ahwMap.get(CRServo.class, "tServo");
        clawServo = ahwMap.get(Servo.class, "cServo");


        FLServo  = ahwMap.get(Servo.class, "FLServo");
        FRServo  = ahwMap.get(Servo.class, "FRServo");

        //sensorColor = ahwMap.get(ColorSensor.class, "sensor_color_distance");

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



        // Set all motors to zero power
        stopAllMotors();

        //Set zero power behavior to braking
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        RobotLog.ii("CAL", "Exit - init");

    }

    //resets the power to zero before starting the action
    public void stopAllMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backleftMotor.setPower(0);
        backrightMotor.setPower(0);
        slide_1.setPower(0);
        slide_2.setPower(0);
        slide_3.setPower(0);

        turnServo.setPower(0);

    }

    public void moveHolonomic(double x, double y , double z)
    {
        double max_power = 0.7;
        double min_power = -1*max_power;

        double fl_power = Range.clip(y + x - z, min_power, max_power);
        double fr_power = Range.clip(y - x + z, min_power, max_power);
        double br_power = Range.clip(y + x + z, min_power, max_power);
        double bl_power = Range.clip(y - x - z, min_power, max_power);

        leftMotor.setPower(fl_power);
        rightMotor.setPower(fr_power);
        backleftMotor.setPower(bl_power);
        backrightMotor.setPower(br_power);

    }

    //extra motions to move slowly go in case we are in a situation like that
    public void forwardSlow() {
        leftMotor.setPower(Range.clip(leftMotor.getPower() + 0.01, 0.3, 1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() + 0.01, 0.3, 1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() + 0.01, 0.3, 1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() + 0.01, 0.3, 1.0));
    }

    public void backwardSlow() {
        leftMotor.setPower(Range.clip(leftMotor.getPower() - 0.01, -0.3, -1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() - 0.01, -0.3, -1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() - 0.01, -0.3, -1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() - 0.01, -0.3, -1.0));
    }

    public void slidesUp(double power)
    {
        slide_1.setPower(power);
        slide_2.setPower(power);
    }

    public void slidesDown(double power)
    {
        slidesUp(-1*power);
    }


    public void slideOut(double power)
    {
        slide_3.setPower(power);
    }

    public void slideIn(double power)
    {
        slideOut(-1*power);
    }

    public void clawTurn1()
    {
        turnServo.setPower(0.5);
    }

    public void clawTurn2()
    {
        turnServo.setPower(-0.5);
    }

    public void claw1()
    {
        clawServo.setPosition(0);
    }

    public void claw2()
    {
        clawServo.setPosition(0.5);
    }

    public void claw3()
    {
        clawServo.setPosition(1);
    }


}