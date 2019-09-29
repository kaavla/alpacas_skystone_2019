package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


enum Direction
{
    FORWARD, BACKWARD, STRAFE_RIGHT, STRAFE_LEFT, ROBOT_UP, ROBOT_DOWN, SPINNER_FORWARD;
}

enum SensorsToUse
{
    NONE, USE_COLOR, USE_DISTANCE, USE_TOUCH;
}


//@Autonomous(name="CallistoAutonomousBase", group="Callisto")
//@Disabled
public class CallistoAutonomousBase extends LinearOpMode
{

    public CallistoHW robot = new CallistoHW();
    public ElapsedTime runtime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0;
    public Direction direction;
    double ref_angle = 0;


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //Need to Change
    static final double ROTATIONS_PER_INCH = 11.42;
    static final double TICKS_PER_INCH = (ROTATIONS_PER_INCH * 1440);
    //Need to Change

    static final double DRIVE_SPEED = 0.7;
    static final double TURN_SPEED = 0.7;
    //maybe change to 0.6

    final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    final String LABEL_GOLD_MINERAL = "Gold Mineral";
    final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //final String VUFORIA_KEY = "AUWa2hP/////AAABmYAEN4JY30WlndAsvgcYjZAcwt/KX4c9VUt+Br3zZxPhIbJ+ovlQrV3YlETOwJ4Q5NajUuwkdpnX2292snWM8iiXzQ2Nm37xl78r82PlDZPAKP8XV+9sBg1KMHO+0zDzTtWNa/fmNPeEhmdff/YWUzcqTmGLnccOhj57waivZa4Y5xDfH4YKssYJUNQKumOd8v5m90IEKYgWghs7BhxpWfQbjzC+3QUKPc7q34V+9W4xQ+2S+hI0inYLrK4rSdiCGU76d8hwlBWuDC8PWrkqwIi6EptTL/nP1rLoWy/Usv6ZUqRRHwkgLNYsrWusN0G5d71F6tdvRDbGdgQKQ2evHWZPPtlVW6u0S5N5S2sXu7R+";
    //final String VUFORIA_KEY = "ARgorO//////AAABmTNryRWHs0iduuKYeo/RTC82lqjhxM1ZpUq78wyOpzAx3mn9q5fKWvlN0WQf9YptFPy17FAlMDA8SQ0qMdUmlmFP7tJ/kTQwXgoPrJFp78KmMTFs+x8vHXPYGkvSM9TY4/2pStDcTHqM3jLl0nlaF+4zulnDcwsz/EKhzL87hkBnatk1GGYTXlyKUB7XrHLDkMyNgtghrk7qdSXnYHhvbQzzZ87d5PTgJygjI7AHUFL4alR8nD9D2tNmP9tkVniV0OQ1ZcVoBQM5fzKPkW7fsZgK/S852yHXx7UWqrD2hiPVNhHKamBc5a4b94uCBCScLaZkiQJQr8a3YxW0otbOpBJzhwNa8AP7exZPXWkLdzw2";
    //final String VUFORIA_KEY = "AUlUrXf/////AAABmWkPvNqe6UDvkpaBp7zgAyEG8NPlcP1sdx3cMoWe6WctHxSusC8WfkZ1hoZwqTpisqtHIF6XaOpZPRd0OG6JBPOXyHCSugA8BYdlRVQzuDIVkKUaVrBC4VqGevEzNnAAmtdAs4UPN9qUvp7G/7ABkfN9B1rt/tPKvXYON2VfszGYPzYebfN3o6sVtCf4iwczH20WIav5MTWjE93uwRqZnW9VK8WNdlUT2dFoUtvyTGSznJhp0cs0sJw5X8pVL5mOahFOVfRcRb/8LMB02xQYplsqv8v/2S5Zn6fme0oZSyzr9Q5QKb9fTtEIMOJWkw86AfZSlUQRlMRzmh8TFT69yRSDTNkm7X7E/0pA7t7ngPf+";
    final String VUFORIA_KEY = "AcG1ToX/////AAABmZwkWkBrt0LBmroz02/Q+Eo1i0bD6c6TFli1v0rZ4J/9xpVacejXAGILhav7QHJDJvUK9FARrzi6G8d0/uNzzdTedJF6IXVXhGdv22fRhU8X02GKixtHW2Bl6RIE5VjUZxdt1RIhK6u6mxZl+53r6gYLDNP9lCocNf6yKWjKMq5Sle6kUoyiguTJzcOyYICrfSUDToefpyBSv0Uf2Iw/055mv/7ZDYLo1arM5HuRnPPkAdvveWQ0ezsJ0cK8Dg1RrpYU7cCa/pbLy1eFj+tKWYLt+XEukfXxWJwwJO39Jciy82SinJ9hQmaIU9H+Y14Q3lkX72hW9l8eQn2XkP/RaGRTgJZOTHBcWWR+euGKekls";
    public TFObjectDetector tfod = null;
    public VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode()
    {
        //Empty Function
    }

    public void initHW()
    {
        RobotLog.ii("CAL", "Enter -  initHW");
        robot.init(hardwareMap);
        initMotorEncoders();
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        else
        {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null)
        {
            tfod.activate();
        }
        telemetry.addData("Path1", "Init HW Done");
        telemetry.update();

        RobotLog.ii("CAL", "Exit -  initHW");
    }

    private void initVuforia()
    {
        RobotLog.ii("CAL", "Enter -  initVuforia");
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        RobotLog.ii("CAL", "Exit -  initVuforia");
        telemetry.addData("Path1", "Init Vuforia Done");
        telemetry.update();


    }

    private void initTfod()
    {
        RobotLog.ii("CAL", "Enter -  initTfod");
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        RobotLog.ii("CAL", "Exit -  initTfod");
        telemetry.addData("Path1", "Init TFOD Done");
        telemetry.update();

    }

    public void myCollectionSlideOut(double power, double timeoutS) {
        robot.MCollectionSlide.setPower(-1 * power);
        runtime.reset();

        while (opModeIsActive() && !isStopRequested() &&
                (runtime.seconds() < timeoutS)) {
            //Do nothing
        }
        robot.MCollectionSlide.setPower(0);
    }

    public void initMotorEncoders()
    {
        RobotLog.ii("CAL", "Enter -  initMotorEncoders");
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MLanderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MLanderLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.ii("CAL", "Exit -  initMotorEncoders");

        telemetry.addData("Path1", "Init MotorEncoders Done");
        telemetry.update();

    }

    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        RobotLog.ii("CAL", "resetAngle - lastAngles = %2.2f", lastAngles.firstAngle);
        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double getAbsoluteAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }



    public void rotate(int degrees, double power)
    {
        RobotLog.ii("CAL", "Enter - rotate - degrees=%d, power=%f",
                degrees, power);

        // restart imu movement tracking.
        resetAngle();

        if (degrees < 0)
        {   // turn right.
            robot.turnRight(power);
        }
        else if (degrees > 0)
        {   // turn left.
            robot.turnLeft(power);
        }
        else return;


        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && !isStopRequested() && getAngle() == 0)
            {
            }

            while (opModeIsActive() && !isStopRequested() && getAngle() > degrees)
            {
            }
        }
        else    // left turn.
            while (opModeIsActive() && !isStopRequested() && getAngle() < degrees)
            {
            }

        // turn the motors off.
        power = 0;
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
        robot.backleftMotor.setPower(power);
        robot.backrightMotor.setPower(power);

        // wait for rotation to stop.
        sleep(50);

        // reset angle tracking on new heading.
        resetAngle();
        RobotLog.ii("CAL", "Exit - rotate");
    }


    public int myTFOD(double timeoutS)
    {
        RobotLog.ii("CAL", "Enter - myTFOD - timeout=%f", timeoutS);
        int positionGold = 2;
        runtime.reset();

        if (tfod != null)
        {
            //tfod.activate();
        }

        while (opModeIsActive() && !isStopRequested() &&
                (runtime.seconds() < timeoutS))
        {
            if (tfod != null)
            {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null)
                {
                    telemetry.addData("Path1", "# Objects Detected %d",updatedRecognitions.size());
                    RobotLog.ii("CAL", "myTFOD - Objects Detected =%d", updatedRecognitions.size());

                    /////////////////////////////////
                    if (updatedRecognitions.size() == 2)
                    {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions)
                        {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                            {
                                goldMineralX = (int) recognition.getLeft();
                            }
                            else if (silverMineral1X == -1)
                            {
                                silverMineral1X = (int) recognition.getLeft();
                            }
                            else
                            {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        // Assuming we see only the center and left (as the camera sees) minerals through the camera
                        // If Gold mineral is not found, that means it is the right (1) one
                        if (goldMineralX == -1)
                        {
                            telemetry.addData("Gold Mineral Position", "Right");
                            positionGold = 1;
                            break;
                        }
                        else if (silverMineral2X == -1)
                        {
                            if (goldMineralX > silverMineral1X)
                            {
                                telemetry.addData("Gold Mineral Position", "Center");
                                positionGold = 2;
                                break;
                            }
                            else
                            {
                                telemetry.addData("Gold Mineral Position", "Left");
                                positionGold = 3;
                                break;
                            }
                        }
                    }
                    telemetry.update();
                    ///////////////////////////////////
                    /*if (updatedRecognitions.size() == 3)
                    {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions)
                        {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                            {
                                goldMineralX = (int) recognition.getLeft();
                            }
                            else if (silverMineral1X == -1)
                            {
                                silverMineral1X = (int) recognition.getLeft();
                            }
                            else
                            {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1)
                        {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
                            {
                                telemetry.addData("Gold Mineral Position", "Left");
                                positionGold = 3;
                                break;
                            }
                            else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                            {
                                telemetry.addData("Gold Mineral Position", "Right");
                                positionGold = 1;
                                break;
                            }
                            else
                            {
                                telemetry.addData("Gold Mineral Position", "Center");
                                positionGold = 2;
                                break;
                            }
                        }
                    }
                    telemetry.update();*/
                }
            }
        }

        if (tfod != null)
        {
            tfod.shutdown();
        }
        RobotLog.ii("CAL", "Exit - myTFOD - positionGold=%d", positionGold);

        return positionGold;
    }


    public void myEncoderDrive(Direction direction, double speed, double Inches, double timeoutS, SensorsToUse sensors_2_use)
    {
        int newLeftTarget = 0;
        int newRightTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        RobotLog.ii("CAL", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);

        //Reset the encoder
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested())
        {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.FORWARD)
            {
                //Go forward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            }
            else if (direction == Direction.BACKWARD)
            {
                //Go backward
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            }
            else if (direction == Direction.STRAFE_RIGHT)
            {
                //Strafe Right
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            }
            else if (direction == Direction.STRAFE_LEFT)
            {
                //Strafe Left
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);

            }
            else
            {
                Inches = 0;
                newLeftTarget = robot.rightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.leftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.backrightMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.backleftMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            }


            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);
            robot.backleftMotor.setTargetPosition(newLeftBackTarget);
            robot.backrightMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
            robot.backleftMotor.setPower(Math.abs(speed));
            robot.backrightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy()))
            {
/*
                if (sensors_2_use == SensorsToUse.USE_COLOR)
                {

                }
                if (sensors_2_use == SensorsToUse.USE_DISTANCE)
                {
                    if (robot.sensorRange.getDistance(DistanceUnit.INCH) < 5) {
                        int count = 0;
                        while (opModeIsActive() && !isStopRequested() &&
                                (runtime.seconds() < timeoutS) &&
                                (count < 20) &&
                                (robot.sensorRange.getDistance(DistanceUnit.INCH) < 5)) {
                            sleep(100);
                            count++;
                        }
                    }

                }

                if (sensors_2_use == SensorsToUse.USE_TOUCH)
                {
                    robot.digitalTouch.setMode(DigitalChannel.Mode.INPUT);

                    if (robot.digitalTouch.getState() == true) {
                        telemetry.addData("Digital Touch", "Is Not Pressed");
                    } else {
                        telemetry.addData("Digital Touch", "Is Pressed");
                        break;
                    }
                    telemetry.update();

                }
                */

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.backleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(50);   // optional pause after each move
        }
        RobotLog.ii("CAL", "Exit - myEncoderDrive ");

    }

    public void myLanderLift(Direction direction,
                             double speed,
                             double Inches,
                             double timeoutS)
    {
        int newLiftTarget;
        RobotLog.ii("CAL", "Enter - myLanderLift");


        //Reset the encoder
        robot.MLanderLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested())
        {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.ROBOT_UP)
            {
                newLiftTarget = robot.MLanderLift.getCurrentPosition() + (int) (Inches * TICKS_PER_INCH);
            }
            else if (direction == Direction.ROBOT_DOWN)
            {
                newLiftTarget = robot.MLanderLift.getCurrentPosition() + (int) (-1 * Inches * TICKS_PER_INCH);
            }
            else
            {
                Inches = 0;
                newLiftTarget = robot.MLanderLift.getCurrentPosition() + (int) (Inches * TICKS_PER_INCH);
            }

            robot.MLanderLift.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.MLanderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.MLanderLift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            //while (opModeIsActive() && !isStopRequested()  &&
            //        (runtime.seconds() < timeoutS) &&
            //       (robot.MLanderLift.isBusy() )) {
            RobotLog.ii("CAL", "Enter - myLanderLift - waiting to get to pos");
            while (opModeIsActive() && !isStopRequested() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.MLanderLift.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1", "Run time to %7f", runtime.seconds());
                //telemetry.addData("Path2", "Running at %7d :%7d",
                //        robot.MLanderLift.getCurrentPosition());
                telemetry.update();
            }
            RobotLog.ii("CAL", "Enter - myLanderLift - reached pos");

            // Stop all motion;
            robot.MLanderLift.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.MLanderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotLog.ii("CAL", "Exit - myLanderLift");

            //sleep(50);   // optional pause after each move
        }
    }

    public void turnspinnerservoforward(double power,
                                        double timoutS)
    {
        robot.spinnerServo.setPower(power);
    }


    public void myDetectionTest(int position,
                                double speed,
                                double timeoutS)
    {
        double curr_angle ;
        RobotLog.ii("CAL", "Enter - myDetectionTest");
        telemetry.addData("Path1", "Position Detected %d", position);
        telemetry.update();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested())
        {
            myLanderLift(Direction.ROBOT_DOWN, 1, 6.5, 4.0);

            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 8, 10.0, SensorsToUse.NONE);
            curr_angle = getAngle();
            telemetry.addData("status", "curr_angle = %f", curr_angle);
            telemetry.update();

            rotate((int)((-1)*(curr_angle - ref_angle )), 0.3);
            // Determine new target position, and pass to motor controller
            if (position == 1)
            {
                myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 12, 10.0, SensorsToUse.NONE);
                //double current_angle = getAbsoluteAngle();
                //rotate((int)(REFERENCE_ANGLE - current_angle), TURN_SPEED);
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 15, 10.0, SensorsToUse.NONE);
                rotate(76, TURN_SPEED);
                //myEncoderDrive(Direction.FORWARD, TURN_SPEED, 13, 10.0);
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 11, 10.0, SensorsToUse.NONE);
            }
            else if (position == 3)
            {
                myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 12, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 16, 10.0, SensorsToUse.NONE);
                rotate(73, TURN_SPEED);
                myEncoderDrive(Direction.FORWARD, TURN_SPEED, 13, 10.0, SensorsToUse.NONE);
            }
            else // Position = 2 and default position
            {
                myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 12, 10.0, SensorsToUse.NONE);
                //myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 2, 10.0);
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 4, 10.0, SensorsToUse.NONE);
                rotate(82, TURN_SPEED);
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 15, 10.0, SensorsToUse.NONE);
            }
        }
        RobotLog.ii("CAL", "Exit - myDetectionTest");

    }

    public void myCollectionLiftDown(double power, double timeoutS)
    {
        RobotLog.ii("CAL", "Enter - myCollectionLiftDown - waiting to get to pos");
        runtime.reset();
        robot.collectionLiftDown(power);
        while (opModeIsActive() && !isStopRequested() &&
                (runtime.seconds() < timeoutS))
        {

            // Display it for the driver.
            telemetry.addData("Path1", "Run time to %7f", runtime.seconds());
            //telemetry.addData("Path2", "Running at %7d :%7d",
            //        robot.MLanderLift.getCurrentPosition());
            telemetry.update();
        }
        robot.collectionLiftDown(0);
        RobotLog.ii("CAL", "Enter - myCollectionLiftDown - reached pos");

    }


}