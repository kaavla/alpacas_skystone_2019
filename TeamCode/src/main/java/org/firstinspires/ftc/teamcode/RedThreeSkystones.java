package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;
@Autonomous(name="AutoRed3Stones", group="JARVIS")

public class RedThreeSkystones extends JARVISAutonomousBase{
    static final int SIDE = 1; //Left side claw

    @Override
    public void runOpMode() {
        RobotLog.ii("CAL", "Enter  - runOpMode - JARVIS Autonomous 1");
        initHW(); //initialize hardware
        ref_angle = getAngle(); //get the current angle and make it the reference angle for the rest of the program

        // Send a telemetry message to signifyrobot waiting;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        //run the function that actually moves the robot
        myDetectionRun(40.0);
        RobotLog.ii("CAL", "Exit - runOpMode - JARVIS Autonomous 1");
    }

    public void correctAngle()
    {
        double currentAngle = 0;
        currentAngle = getAngle(); //get the current angle

        //subtract the reference angle from the current angle and rotate that many degrees
        //so that the robot is aligned with the starting position
        rotate((int)((-1)*(currentAngle - ref_angle)), 0.2);
        sleep(100);
    }

    public void getStone()
    {
        //strafe away from the skystone so we have space to pick it up
        //myEncoderDrive(JARVISAutonomousBase.Direction.STRAFE_LEFT, 0.2, 3, 5.0, JARVISAutonomousBase.SensorsToUse.NONE);
        //robot.openGrabberClaw(SIDE); //open the claw
        //sleep(100);
        //robot.setGrabberHalfDown(SIDE); //put the claw half down
        //sleep(100);
        //strafe closer to the skystone so we can pick it up
        //myEncoderDrive(JARVISAutonomousBase.Direction.STRAFE_RIGHT, DRIVE_SPEED,3, 5.0, JARVISAutonomousBase.SensorsToUse.NONE);
        sleep(100);
        robot.setGrabberDown(SIDE); //put the claw all the way down on top of the stone
        sleep(500);
        robot.closeGrabberClaw(SIDE); //close the claw, holding the stone
        sleep(500);
        robot.setGrabberUp(SIDE); //put the claw up so that it is vertical and doesn't drag on the ground
        sleep(300);
        //strafe towards the wall a little bit so we don't collide with the bridge
        myEncoderDrive(JARVISAutonomousBase.Direction.STRAFE_LEFT, DRIVE_SPEED,4, 5.0, JARVISAutonomousBase.SensorsToUse.NONE);
        sleep(100);
    }

    public void releaseStone()
    {
        robot.setGrabberDown(SIDE); //put the claw down
        sleep(100);
        robot.openGrabberClaw(SIDE); //open the claw, letting go of the skystone
        sleep(100);
        robot.setGrabberUp(SIDE); //put the claw up
        sleep(100);
        robot.closeGrabberClaw(SIDE); //close the claw so it doesn't hit the top of the bridge
        sleep(100);
    }

    public void myDetectionRun(double timeoutS)
    {
        RobotLog.ii("CAL", "Enter i- myDetectionRun");
        //a variable that holds the number of inches we move from the stone closest to the bridge
        //to the skystone so we know how much extra we need to move to always end up the
        //same distance on the other side of the bridge
        double strafe_back_previous = 0;
        //same as above except for the second skystone
        double strafe_back = 0;

        //initialize the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {
            robot.openGrabberClaw(SIDE); //open the claw
            sleep(100);
            robot.setGrabberHalfDown(SIDE); //put the claw half down
            sleep(100);
            //MOve towards the skystones using a distance sensor so we don't collide with them
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.2, 50, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);
            //align with the reference angle
            correctAngle();

            //if the first stone we see is NOT a skystone, continue to move forward while sensing
            //stop whenever the color sensed is not yellow, but black (skystone)
            //add the extra distance traveled using the color sensor to strafe_back_previous
            if (myDetectSkystone(SideToUse.USE_RIGHT, 10) == false) {
                myEncoderDrive(Direction.FORWARD, 0.1, 24, 10.0, SensorsToUse.USE_COLOR_RIGHT);
                strafe_back_previous = distance_traveled;
                telemetry.addData("strafe back = ", strafe_back_previous);
                telemetry.update();
                //go backward an inch to be sure that we're aligned with the middle of the skystone
                myEncoderDrive(Direction.BACKWARD, 0.1, 1, 5.0, SensorsToUse.NONE);
            }

            //Grab the skystone and go to the other side of the bridge
            getStone();
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 35 + strafe_back_previous, 10.0, SensorsToUse.NONE);

            //drop the skystone
            releaseStone();



            //if (strafe_back_previous < 5) {
            if (strafe_back_previous < 12) {
                //Drive back to collect the second stone
                myEncoderDrive(Direction.FORWARD, 0.6, 44 + strafe_back_previous, 10.0, SensorsToUse.NONE);

                robot.openGrabberClaw(SIDE); //open the claw
                sleep(100);
                robot.setGrabberHalfDown(SIDE); //put the claw half down
                sleep(100);
                correctAngle(); //correct angle to match the reference angle
                //Drive till we are close to the stone again
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,24, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);

                //move forward while sensing using the color sensor
                //stop whenever the color sensed is not yellow, but black (skystone)
                //add the extra distance traveled using the color sensor to strafe_back
                myEncoderDrive(Direction.FORWARD, 0.1, 20, 5.0, SensorsToUse.USE_COLOR_RIGHT);
                strafe_back = distance_traveled;
                telemetry.addData("strafe back = ", strafe_back);
                telemetry.update();

                //Grab the skystone
                myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,2, 5.0, SensorsToUse.NONE);
                getStone();
                //sec0/ond time, we need to strafe an extra inch to avoid the bridge
                //myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,3, 5.0, SensorsToUse.NONE);
                correctAngle(); //correct angle to match the reference angle

                //drive to other side and drop the stone
                myEncoderDrive(Direction.BACKWARD, 0.5, 52 + strafe_back_previous + strafe_back, 10.0, SensorsToUse.NONE);
                releaseStone();
                myEncoderDrive(Direction.FORWARD, 0.6, 48 + strafe_back_previous, 10.0, SensorsToUse.NONE);
                robot.openGrabberClaw(SIDE); //open the claw
                sleep(100);
                robot.setGrabberHalfDown(SIDE); //put the claw half down
                sleep(100);
                //Drive till we are close to the stone again
                correctAngle(); //correct angle to match the reference angle
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,24, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,2, 5.0, SensorsToUse.NONE);
                //pick up the skystone
                getStone();
                correctAngle(); //correct angle to match the reference angle
                myEncoderDrive(Direction.BACKWARD, 0.6, 38 + strafe_back_previous, 10.0, SensorsToUse.NONE);
                //drop the skystone
                releaseStone();
            } else /*if (strafe_back_previous > 10)*/{
                //Drive back to collect the second stone
                myEncoderDrive(Direction.FORWARD, 0.6, 52 + strafe_back_previous, 10.0, SensorsToUse.NONE);

                robot.openGrabberClaw(SIDE); //open the claw
                sleep(100);
                robot.setGrabberHalfDown(SIDE); //put the claw half down
                sleep(100);
                correctAngle(); //correct angle to match the reference angle
                //Drive till we are close to the stone again
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,24, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);
                myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,2, 5.0, SensorsToUse.NONE);

                //move forward while sensing using the color sensor
                //stop whenever the color sensed is not yellow, but black (skystone)
                //add the extra distance traveled using the color sensor to strafe_back
                //myEncoderDrive(Direction.FORWARD, 0.1, 20, 5.0, SensorsToUse.USE_COLOR_RIGHT);
                //strafe_back = distance_traveled;
                //telemetry.addData("strafe back = ", strafe_back);
                //telemetry.update();

                //Grab the skystone
                myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,2, 5.0, SensorsToUse.NONE);
                getStone();
                //sec0/ond time, we need to strafe an extra inch to avoid the bridge
                //myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,3, 5.0, SensorsToUse.NONE);
                correctAngle(); //correct angle to match the reference angle

                //drive to other side and drop the stone
                myEncoderDrive(Direction.BACKWARD, 0.5, 58 + strafe_back_previous + strafe_back, 10.0, SensorsToUse.NONE);
                releaseStone();
                myEncoderDrive(Direction.FORWARD, 0.6, 18 + strafe_back_previous, 10.0, SensorsToUse.NONE);
                robot.openGrabberClaw(SIDE); //open the claw
                sleep(100);
                robot.setGrabberHalfDown(SIDE); //put the claw half down
                sleep(100);
                correctAngle(); //correct angle to match the reference angle
                //Drive till we are close to the stone again
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,24, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,2, 5.0, SensorsToUse.NONE);
                //pickup the skystone
                getStone();
                correctAngle(); //correct angle to match the reference angle
                myEncoderDrive(Direction.BACKWARD, 0.6, 14 + strafe_back_previous, 10.0, SensorsToUse.NONE);
                //drop the skystone
                releaseStone();
            } /*else {
                //Drive back to collect the second stone
                myEncoderDrive(Direction.FORWARD, 0.6, 44 + strafe_back_previous, 10.0, SensorsToUse.NONE);

                robot.openGrabberClaw(SIDE); //open the claw
                sleep(100);
                robot.setGrabberHalfDown(SIDE); //put the claw half down
                sleep(100);
                correctAngle(); //correct angle to match the reference angle
                //Drive till we are close to the stone again
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,24, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);

                //move forward while sensing using the color sensor
                //stop whenever the color sensed is not yellow, but black (skystone)
                //add the extra distance traveled using the color sensor to strafe_back
                myEncoderDrive(Direction.FORWARD, 0.1, 20, 5.0, SensorsToUse.USE_COLOR_RIGHT);
                strafe_back = distance_traveled;
                telemetry.addData("strafe back = ", strafe_back);
                telemetry.update();

                //Grab the skystone
                myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,2, 5.0, SensorsToUse.NONE);
                getStone();
                //sec0/ond time, we need to strafe an extra inch to avoid the bridge
                //myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,3, 5.0, SensorsToUse.NONE);
                correctAngle(); //correct angle to match the reference angle

                //drive to other side and drop the stone
                myEncoderDrive(Direction.BACKWARD, 0.5, 52 + strafe_back_previous + strafe_back, 10.0, SensorsToUse.NONE);
                releaseStone();
                myEncoderDrive(Direction.FORWARD, 0.6, 28 + strafe_back_previous, 10.0, SensorsToUse.NONE);
                robot.openGrabberClaw(SIDE); //open the claw
                sleep(100);
                robot.setGrabberHalfDown(SIDE); //put the claw half down
                sleep(100);
                correctAngle(); //correct angle to match the reference angle
                //Drive till we are close to the stone again
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,24, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);
                myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,2, 5.0, SensorsToUse.NONE);
                //pickup the skystone
                getStone();
                correctAngle(); //correct angle to match the reference angle
                myEncoderDrive(Direction.BACKWARD, 0.6, 20 + strafe_back_previous, 10.0, SensorsToUse.NONE);
                //drop the skystone
                releaseStone();
            }*/

            //drive under the bridge then strafe towards the bridge so that our alliance also has space to park
            myEncoderDrive(Direction.FORWARD, 0.6, 15, 10.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
            robot.openCapStoneClaw();


        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}