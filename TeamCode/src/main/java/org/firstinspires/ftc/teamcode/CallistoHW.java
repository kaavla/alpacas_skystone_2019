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

    public Servo foundationServo = null;
    public Servo stoneServo = null;

/*
    //public DcMotor MCollectionSlide = null;
    //public DcMotor MCollectionLift = null;
    //public DcMotor MDropLift = null;
    public DcMotor MLanderLift = null;

    public CRServo spinnerServo = null;

    public Servo markerServo = null;

 */


    //public DigitalChannel digitalTouch = null;  // Hardware Device Object
    //public DistanceSensor sensorRange = null;


    //static final double     REFERENCE_ANGLE           = 165;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    public BNO055IMU imu = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        RobotLog.ii("CAL", "Enter - init");

        leftMotor = ahwMap.get(DcMotor.class, "M1");
        rightMotor = ahwMap.get(DcMotor.class, "M2");
        backleftMotor = ahwMap.get(DcMotor.class, "M3");
        backrightMotor = ahwMap.get(DcMotor.class, "M4");
        //foundationServo = ahwMap.get(Servo.class, "foundationServo");
        //collectServo = ahwMap.get(CRServo.class, "stoneServo");



        //collectServo = ahwMap.get(Servo.class, "foundationServo");

/*
        //MCollectionSlide = ahwMap.get(DcMotor.class, "MCollectionSlide");
        MCollectionLift = ahwMap.get(DcMotor.class, "MCollectionLift");
        MDropLift = ahwMap.get(DcMotor.class, "MDropLift");
        MLanderLift = ahwMap.get(DcMotor.class, "MLanderLift");

        spinnerServo = ahwMap.get(CRServo.class, "spinnerServo");
        markerServo = ahwMap.get(Servo.class, "markerServo");

        digitalTouch = ahwMap.get(DigitalChannel.class, "sensor_digital");
        //sensorRange = ahwMap.get(DistanceSensor.class, "sensor_range");

 */


    public void moveHolonomic(double x, double y , double z) {
    {
        double max_power = 0.7;
        double min_power = -1*max_power;

        double fl_power = Range.clip(y + x, -1.0, 1.0);
        double fr_power = Range.clip(y - x, -1.0, 1.0);
        double br_power = Range.clip(y + x, -1.0, 1.0);
        double bl_power = Range.clip(y - x, -1.0, 1.0);

        leftMotor.setPower(fl_power);
        rightMotor.setPower(fr_power);
        backleftMotor.setPower(bl_power);
        backrightMotor.setPower(br_power);
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
    public void turnTraytocollect()
    {
        foundationServo.setPosition(0);
    }

    public void turnTraytodrop()
    {
        foundationServo.setPosition(0.85);
    }
    public void turnClawtocollect()
    {
        stoneServo.setPosition(0);
    }
    public void turnClawotoDrop()
    {
        stoneServo.setPosition(0.85);
    }






}
    }