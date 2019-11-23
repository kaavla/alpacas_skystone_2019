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

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    public BNO055IMU imu = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        RobotLog.ii("CAL", "Enter - init");


        leftMotor = ahwMap.get(DcMotor.class, "M1"); //FL
        rightMotor = ahwMap.get(DcMotor.class, "M2"); //FR
        backleftMotor = ahwMap.get(DcMotor.class, "M3"); //BL
        backrightMotor = ahwMap.get(DcMotor.class, "M4"); //BR


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
        //MLanderLift.setDirection(DcMotorSimple.Direction.FORWARD);
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
    }

    public void moveHolonomic(double x, double y , double z)
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
