package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;
@Autonomous(name="Shank Auto Test", group="JARVIS")

public class ShankAutoTest extends JARVISAutonomousBase {


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
            myEncoderSlide1(Direction.SLIDE_UP, DRIVE_SPEED, 6, 2, SensorsToUse.NONE);
            robot.rotateClawInline();
            robot.openClaw();

            //MOve towards the skystones
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 24, 5.0, SensorsToUse.NONE);

            if (myDetectSkystone(10) == false) {
                //detected stone. Strafe left to test the next one.
                myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 8, 5.0, SensorsToUse.NONE);

                strafe_back = strafe_back + 8;
                if (myDetectSkystone(10) == false) {
                    //detected stone. Strafe left to test the next one.
                    myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 8, 5.0, SensorsToUse.NONE);
                    strafe_back = strafe_back + 8;
                }
            }

            //Strafe the opposite direction so claw is in middle
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 1, 5.0, SensorsToUse.NONE);

            //Get slide Out so claw is on top of skystone
            myEncoderInOutSlide(Direction.SLIDE_OUT, DRIVE_SPEED, 9, 5, SensorsToUse.NONE);
            //Get the slide down to collect the skystone by closing the claw
            myEncoderSlide1(Direction.SLIDE_DOWN, DRIVE_SPEED, 2, 1.5, SensorsToUse.NONE);
            robot.closeClaw();

            //Move the slide up and linear slide in
            myEncoderSlide(Direction.SLIDE_UP, DRIVE_SPEED, 8, 2.5, SensorsToUse.NONE);
            //myEncoderInOutSlide(Direction.SLIDE_IN, DRIVE_SPEED, 4, 5, SensorsToUse.NONE);


            //after these too we assume that the skystone is the third and it will play out the code below

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
            //myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            //move back to same position
            /*
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 36 + strafe_back + 16, 15.0, SensorsToUse.NONE);

            rotate(-90, 0.2);

            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 1, 5.0, SensorsToUse.NONE);

            //Get slide Out so claw is on top of skystone
            myEncoderInOutSlide(Direction.SLIDE_OUT, DRIVE_SPEED, 9, 5, SensorsToUse.NONE);
            //Get the slide down to collect the skystone by closing the claw
            myEncoderSlide1(Direction.SLIDE_DOWN, DRIVE_SPEED, 2, 1.5, SensorsToUse.NONE);
            robot.closeClaw();

            //Move the slide up and linear slide in
            myEncoderSlide(Direction.SLIDE_UP, DRIVE_SPEED, 8, 3, SensorsToUse.NONE);
            //myEncoderInOutSlide(Direction.SLIDE_IN, DRIVE_SPEED, 4, 5, SensorsToUse.NONE);


            //after these too we assume that the skystone is the third and it will play out the code below

            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            rotate(90, 0.2);
            robot.openClaw();

             */



            /*
            // Slides need to go down in oder to pass under the bridge
            myEncoderSlide1(Direction.SLIDE_DOWN, DRIVE_SPEED, 2, 1, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 72, 15.0, SensorsToUse.NONE);
            robot.openClaw();
            */

            /*
            //Move Slide Up and open Claw
            myEncoderSlide1(Direction.SLIDE_UP, DRIVE_SPEED, 6, 2, SensorsToUse.NONE);
            robot.rotateClawInline();
            robot.openClaw();

            //Get slide Out so claw is on top of skystone
            myEncoderInOutSlide(Direction.SLIDE_OUT, DRIVE_SPEED, 8, 5, SensorsToUse.NONE);
            //Get the slide down to collect the skystone by closing the claw
            myEncoderSlide1(Direction.SLIDE_DOWN, DRIVE_SPEED, 2, 1.5, SensorsToUse.NONE);
            //robot.closeClaw();

             */
        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}
