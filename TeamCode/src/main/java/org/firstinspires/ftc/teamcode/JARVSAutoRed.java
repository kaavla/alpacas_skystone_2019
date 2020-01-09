package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;
@Autonomous(name="Jarvis Auto Red", group="JARVIS")

public class JARVSAutoRed extends JARVISAutonomousBase {


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
            //myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 36, 5.0, SensorsToUse.NONE);


            //Move Slide Up and open Claw
            //myEncoderSlide1(Direction.SLIDE_UP, DRIVE_SPEED, 6, 2, SensorsToUse.NONE);
            //robot.rotateClawInline();
            //robot.openClaw();

            //MOve towards the skystones

            myEncoderDrive(Direction.STRAFE_RIGHT, 0.3, 21, 5.0, SensorsToUse.NONE);
            double currentAngle;
            currentAngle = getAngle();
            rotate((int)((-1)*(currentAngle - ref_angle)), 0.2);
            sleep(300);
            if (myDetectSkystone(10) == false) {
                //detected stone. Strafe left to test the next one.
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 7, 5.0, SensorsToUse.NONE);

                strafe_back = strafe_back + 7;
                if (myDetectSkystone(10) == false) {
                    //detected stone. Strafe left to test the next one.
                    myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 7, 5.0, SensorsToUse.NONE);
                    strafe_back = strafe_back + 7;
                }
            }

            //myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 2, 5.0, SensorsToUse.NONE);
            //myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 2, 5.0, SensorsToUse.NONE);

            sleep(300);
            robot.openGrabberClaw(1);
            sleep(500);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,2, 5.0, SensorsToUse.NONE);
            sleep(500);
            robot.setGrabberDown(1);
            sleep(500);
            robot.closeGrabberClaw(1);
            sleep(500);
            robot.setGrabberUp(1);
            sleep(500);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
            sleep(250);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 35 + strafe_back, 10.0, SensorsToUse.NONE);
            getAngle();
            currentAngle = getAngle();
            rotate((int)((-1)*(currentAngle - ref_angle)), 0.2);
            sleep(250);
            robot.setGrabberDown(1);
            sleep(250);
            robot.openGrabberClaw(1);
            sleep(250);
            robot.setGrabberUp(1);
            sleep(250);
            robot.closeGrabberClaw(1);
            sleep(250);
            myEncoderDrive(Direction.STRAFE_LEFT,DRIVE_SPEED,5,5.0,SensorsToUse.NONE);
            getAngle();
            currentAngle = getAngle();
            rotate((int)((-1)*(currentAngle - ref_angle)), 0.2);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 65 + strafe_back, 10.0, SensorsToUse.NONE);
            getAngle();
            currentAngle = getAngle();
            rotate((int)((-1)*(currentAngle - ref_angle)), 0.2);
            sleep(250);
            //myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,3, 5.0, SensorsToUse.NONE);
            sleep(500);
            robot.openGrabberClaw(1);
            sleep(500);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
            sleep(500);
            robot.setGrabberDown(1);
            sleep(500);
            robot.closeGrabberClaw(1);
            sleep(500);
            robot.setGrabberUp(1);
            sleep(500);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
            sleep(250);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 65 + strafe_back, 10.0, SensorsToUse.NONE);
            getAngle();
            currentAngle = getAngle();
            rotate((int)((-1)*(currentAngle - ref_angle)), 0.2);
            sleep(250);
            myEncoderDrive(Direction.STRAFE_RIGHT,DRIVE_SPEED,2,5.0,SensorsToUse.NONE);
            sleep(250);
            robot.setGrabberDown(1);
            sleep(250);
            robot.openGrabberClaw(1);
            sleep(250);
            robot.setGrabberUp(1);
            sleep(250);
            robot.closeGrabberClaw(1);
            sleep(250);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 14, 10.0, SensorsToUse.NONE);
            getAngle();
            currentAngle = getAngle();
            rotate((int)((-1)*(currentAngle - ref_angle)), 0.2);
            sleep(250);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,3, 5.0, SensorsToUse.NONE);
            /*
            robot.GrabberLeftClawServo.setPosition(0.5);
            robot.GrabberLeftTurnServo.setPosition(0.4);
            robot.GrabberLeftClawServo.setPosition(0);
            robot.GrabberLeftTurnServo.setPosition(0);

            robot.GrabberRightClawServo.setPosition(0.5);
            robot.GrabberRightTurnServo.setPosition(0.4);
            robot.GrabberRightClawServo.setPosition(0);
            robot.GrabberRightTurnServo.setPosition(0);

            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 2, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 40 + strafe_back, 10.0, SensorsToUse.NONE);
            openGrabberClawAuto(0);

            //Strafe the opposite direction so claw is in middle
            //myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
            //myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 1, 5.0, SensorsToUse.NONE);

            //Get slide Out so claw is on top of skystone
            //myEncoderInOutSlide(Direction.SLIDE_OUT, DRIVE_SPEED, 9, 5, SensorsToUse.NONE);
            //Get the slide down to collect the skystone by closing the claw
            //myEncoderSlide1(Direction.SLIDE_DOWN, DRIVE_SPEED, 2, 1.5, SensorsToUse.NONE);
            //robot.closeClaw();

            //Move the slide up and linear slide in
            //myEncoderSlide(Direction.SLIDE_UP, DRIVE_SPEED, 8, 2.5, SensorsToUse.NONE);
            //myEncoderInOutSlide(Direction.SLIDE_IN, DRIVE_SPEED, 4, 5, SensorsToUse.NONE);


            //after these too we assume that the skystone is the third and it will play out the code below
        /*
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            rotate(90, 0.2);
            sleep(500);
            myEncoderDrive(Direction.STRAFE_LEFT, 0.3, 10, 5.0, SensorsToUse.NONE);
            robot.openClaw();
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 4, 5, SensorsToUse.NONE);
            robot.closeClaw();
            myEncoderInOutSlide(Direction.SLIDE_IN, DRIVE_SPEED, 8, 5, SensorsToUse.NONE);
            //Get the slide down to collect the skystone by closing the claw
            myEncoderSlide1(Direction.SLIDE_DOWN, DRIVE_SPEED, 2, 2.5, SensorsToUse.NONE);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 36 + strafe_back, 15.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 12, 5.0, SensorsToUse.NONE);
            //move back to same position
            /*
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 52, 15.0, SensorsToUse.NONE);
            //robot.claw1();
            //sleep(1000);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 75, 15.0, SensorsToUse.NONE);
            //mySlidesAuto(0.3, 5.0);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 5, 5.0, SensorsToUse.NONE);
            //mySlidesAuto(-0.3, 5.0);
            sleep(1000);
            robot.closeClaw();
            sleep(1000);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 72, 10.0, SensorsToUse.NONE);
            robot.openClaw();
            sleep(1000);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 19, 10.0, SensorsToUse.NONE);
*/

        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}
