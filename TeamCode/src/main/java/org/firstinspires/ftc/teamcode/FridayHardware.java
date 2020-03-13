package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FridayHardware
{
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor backrightMotor = null;
    public DcMotor backleftMotor = null;

    public DistanceSensor sensorDistanceRight = null;
    public DistanceSensor sensorDistanceLeft = null;
    public DistanceSensor sensorDistanceFL = null;
    // public ColorSensor sensorColor = null;
    public ColorSensor sensorColorLeft = null;
    public ColorSensor sensorColorRight = null;
    public TouchSensor touch_sensor = null;

    public BNO055IMU imu = null;

    Orientation lastAngles = new Orientation();  //?
    double globalAngle, power = .40, correction;  //?

    //sets the power used in each of the actions

    /* Initialize standard Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        RobotLog.ii("CAL", "Enter - init");

        leftMotor = ahwMap.get(DcMotor.class, "LF");
        rightMotor = ahwMap.get(DcMotor.class, "RF");
        backleftMotor = ahwMap.get(DcMotor.class, "LB");
        backrightMotor = ahwMap.get(DcMotor.class, "RB");

        sensorColorLeft = ahwMap.get(ColorSensor.class, "sensor_color_left");
        sensorColorRight = ahwMap.get(ColorSensor.class, "sensor_color_right");

        sensorDistanceLeft = ahwMap.get(DistanceSensor.class, "sensorDistanceLeft");
        sensorDistanceRight = ahwMap.get(DistanceSensor.class, "sensorDistanceRight");
        sensorDistanceFL = ahwMap.get(DistanceSensor.class, "sensorDistanceFL");

        touch_sensor = ahwMap.get(TouchSensor.class, "touch_sensor");

        imu = ahwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RobotLog.ii("CAL", "Exit - init");
    }

    public void stopAllMotors() {
        RobotLog.ii("CAL", "Stopping All motors");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backleftMotor.setPower(0);
        backrightMotor.setPower(0);
    }

    public void initMotorNoEncoders() {
        RobotLog.ii("CAL", "Enter -  initMotorNoEncoders");

        // Sets the mode of the motors to run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RobotLog.ii("CAL", "Exit -  initMotorNoEncoders");
    }


    public void initMotorEncoders() {
        RobotLog.ii("CAL", "Enter -  initMotorEncoders");

        // Sets the mode of the motors to run WITH encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RobotLog.ii("CAL", "Exit -  initMotorEncoders");
    }



    public void moveHolonomic(double x, double y , double z)
    {
        double max_power = 0.6;
        double min_power = -1*max_power;

        double fl_power = Range.clip(y + x - z, min_power, max_power);
        double fr_power = Range.clip(y - x + z, min_power, max_power);
        double br_power = Range.clip(y + x + z, min_power, max_power);
        double bl_power = Range.clip(y - x - z, min_power, max_power);

        // Sets the power of the motors to the power defined above
        leftMotor.setPower(fl_power);
        rightMotor.setPower(fr_power);
        backleftMotor.setPower(bl_power);
        backrightMotor.setPower(br_power);
    }

    //extra motions to move slowly go in case we are in a situation like that
    public void forwardSlow() {
        leftMotor.setPower(Range.clip(leftMotor.getPower() + 0.01, 0.5, 1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() + 0.01, 0.5, 1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() + 0.01, 0.5, 1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() + 0.01, 0.5, 1.0));
    }

    public void backwardSlow() {
        leftMotor.setPower(Range.clip(leftMotor.getPower() - 0.02, -0.5, -1.0));
        rightMotor.setPower(Range.clip(rightMotor.getPower() - 0.02, -0.5, -1.0));
        backleftMotor.setPower(Range.clip(backleftMotor.getPower() - 0.02, -0.5, -1.0));
        backrightMotor.setPower(Range.clip(backrightMotor.getPower() - 0.02, -0.5, -1.0));
    }
}
