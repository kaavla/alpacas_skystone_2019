package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;
@Autonomous(name="Jarvis Auto Test 3", group="JARVIS")

public class JARVISAutonomous1 extends JARVISAutonomousBase {


    @Override
    public void runOpMode() {
        RobotLog.ii("CAL", "Enter  - runOpMode - JARVIS Autonomous 1");

        initHW();

        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();

        // Send telemetry message to signifyrobot waiting;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        myDetectionRun(40.0);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        RobotLog.ii("CAL", "Exit - runOpMode - JARVIS Autonomous 1");

    }

    public void myDetectionRun(double timeoutS)
    {

        RobotLog.ii("CAL", "Enter - myDetectionRun");
        double strafe_back = 0;

        robot.initMotorNoEncoders();
        //myTFOD(30.0);

        //initialized the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            //Move Slide Up and open Claw
            myEncoderSlide(Direction.SLIDE_UP, DRIVE_SPEED, 2, 5, SensorsToUse.NONE);
            robot.openClaw();

            //MOve towards the skystones
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 24, 5.0, SensorsToUse.NONE);

            if (myDetectSkystone(10) == false) {
                //detected stone. Strafe left to test the next one.
                myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
                strafe_back = strafe_back + 6;
                if (myDetectSkystone(10) == false) {
                    //detected stone. Strafe left to test the next one.
                    myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
                    strafe_back = strafe_back + 6;
                }
            }

            //Strafe the opposite direction so claw is in middle
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,7, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 1, 5.0, SensorsToUse.NONE);

            //Get slide Out so claw is on top of skystone
            myEncoderInOutSlide(Direction.SLIDE_OUT, DRIVE_SPEED, 4, 5, SensorsToUse.NONE);
            //Get the slide down to collect the skystone by closing the claw
            myEncoderSlide(Direction.SLIDE_DOWN, DRIVE_SPEED, 2, 5, SensorsToUse.NONE);
            robot.closeClaw();

            //Move the slide up and linear slide in
            myEncoderSlide(Direction.SLIDE_UP, DRIVE_SPEED, 2, 5, SensorsToUse.NONE);
            myEncoderInOutSlide(Direction.SLIDE_IN, DRIVE_SPEED, 4, 5, SensorsToUse.NONE);


            //after these too we assume that the skystone is the third and it will play out the code below
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 2, 5.0, SensorsToUse.NONE);
            //myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 72, 15.0, SensorsToUse.NONE);

            //myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 96, 15.0, SensorsToUse.NONE);
            //myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);

//after these too we assume that the skystone is the third and it will play out the code below
            //robot.claw1();
            //sleep(1000);
            //myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 30, 10,SensorsToUse.NONE);
            mySlideAuto(0.3, 0.6);
            sleep(1000);
            //robot.clawTurn1();
            //sleep(1000);
            //mySlidesAuto(0.3, 4.0);
            //sleep(1000);
            //robot.claw3();
            //sleep(1000);
            //mySlidesAuto(-0.3, 1.0);
            //sleep(1000);
            mySlideAuto(-0.3, 0.6);
            sleep(1000);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 52, 15.0, SensorsToUse.NONE);
            //robot.claw1();
            //sleep(1000);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 75, 15.0, SensorsToUse.NONE);
            mySlidesAuto(0.3, 5.0);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 5, 5.0, SensorsToUse.NONE);
            mySlidesAuto(-0.3, 5.0);
            sleep(1000);
            robot.closeClaw();
            sleep(1000);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 72, 10.0, SensorsToUse.NONE);
            robot.openClaw();
            sleep(1000);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 19, 10.0, SensorsToUse.NONE);

        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}
