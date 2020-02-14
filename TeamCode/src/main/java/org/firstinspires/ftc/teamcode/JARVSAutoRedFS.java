package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Jarvis Auto Red FS", group="JARVIS")

public class JARVSAutoRedFS extends JARVISAutonomousBase {
    static final int SIDE = 1; //Left side claw

    @Override
    public void runOpMode() {
        RobotLog.ii("CAL", "Enter  - runOpMode - JARVIS Autonomous 1");
        initHW(); //initialize hardware
        ref_angle = getAngle(); //get the current angle and make it the reference angle for the rest of the program
        ref_angle_1 = getAngle() - 90;
        moveFoundationServoUp();

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
        sleep(200);
    }

    public void correctAngleX()
    {
        double currentAngle1 = 0;
        currentAngle1 = getAngle(); //get the current angle

        //subtract the reference angle from the current angle and rotate that many degrees
        //so that the robot is aligned with the starting position
        rotate((int)((-1)*(currentAngle1 - ref_angle_1)), 0.3);
        sleep(200);
    }

    public void getStone()
    {
        //strafe away from the skystone so we have space to pick it up
        myEncoderDrive(Direction.STRAFE_LEFT, 0.2, 3, 5.0, SensorsToUse.NONE);
        robot.openGrabberClaw(SIDE); //open the claw
        sleep(100);
        robot.setGrabberHalfDown(SIDE); //put the claw half down
        sleep(100);
        //strafe closer to the skystone so we can pick it up
        myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,3, 5.0, SensorsToUse.NONE);
        sleep(200);
        robot.setGrabberDown(SIDE); //put the claw all the way down on top of the stone
        sleep(500);
        robot.closeGrabberClaw(SIDE); //close the claw, holding the stone
        sleep(500);
        robot.setGrabberUp(SIDE); //put the claw up so that it is vertical and doesn't drag on the ground
        sleep(500);
        //strafe towards the wall a little bit so we don't collide with the bridge
        myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,4, 5.0, SensorsToUse.NONE);
        sleep(100);
    }

    public void releaseStone()
    {
        robot.setGrabberDown(SIDE); //put the claw down
        sleep(150);
        robot.openGrabberClaw(SIDE); //open the claw, letting go of the skystone
        sleep(200);
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
        //same as above but for the foundation
        double strafe_forward = 0;

        //initialize the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {
            //MOve towards the skystones using a distance sensor so we don't collide with them
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.2, 50, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);
            //align with the reference angle
            correctAngle();

            //if the first stone we see is NOT a skystone, continue to move forward while sensing
            //stop whenever the color sensed is not yellow, but black (skystone)
            //add the extra distance traveled using the color sensor to strafe_back_previous
            if (myDetectSkystone(SideToUse.USE_RIGHT, 10) == false) {
                myEncoderDrive(Direction.BACKWARD, 0.1, 24, 10.0, SensorsToUse.USE_COLOR_RIGHT);
                strafe_back_previous = distance_traveled;
                telemetry.addData("strafe back = ", strafe_back_previous);
                telemetry.update();
                //go backward an inch to be sure that we're aligned with the middle of the skystone
                myEncoderDrive(Direction.FORWARD, 0.1, 1, 5.0, SensorsToUse.NONE);
            }

            //Grab the skystone and go to the other side of the bridge
            getStone();
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 84 + strafe_back_previous,
                    10.0, SensorsToUse.NONE);

            //move closer to the foundation
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.3, 5, 20, SensorsToUse.NONE);

            /*if (myDetectFoundation(SideToUse.USE_RIGHT, 10) == false) {
                myEncoderDrive(Direction.BACKWARD, 0.2, 10, 20, SensorsToUse.USE_COLOR_RIGHT);
                strafe_forward = distance_traveled;
                telemetry.addData("strafe forward = ", strafe_forward);
                telemetry.update();
                myEncoderDrive(Direction.BACKWARD, 0.2, 10, 20, SensorsToUse.NONE);
            }*/

            //Move close to the foundation so we can drop the skystone on well.
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.3, 3, 20, SensorsToUse.USE_DISTANCE_RIGHT_BLD);

            //correct the angle so we are parallel to the foundation.
            correctAngle();
            //drop the skystone
            releaseStone();

            //move away from the foundation so we don't hit it
            myEncoderDrive(Direction.STRAFE_LEFT, 0.3, 3, 20, SensorsToUse.NONE);
            //turn so the servos are in line with the foundation
            rotate(-90, 0.3);
            //correct the angle
            correctAngle();
            //move towards it
            myEncoderDrive(Direction.FORWARD, 0.2, 5, 20, SensorsToUse.NONE);
            //go slowly toward the foundation so it can lock on well
            myEncoderDrive(Direction.FORWARD, 0.1, 3, 20, SensorsToUse.NONE);
/////////////////////////////////////////////////////////////////////////////////////////////////////
            sleep(100);
            //move the foundation attachment down
            moveFoundationServoDown();
            //leave time for the foundation servos to move
            sleep(350);

            //back up to be close to the wall
            myEncoderDrive(Direction.BACKWARD, 0.2, 20, 5, SensorsToUse.NONE);
            //turn the foundation so it is parallel to the front wall
            rotate(-90, 0.6);
            //move forwards with the foundation and bring it close to the wall
            myEncoderDrive(Direction.FORWARD, 0.2, 7, 5, SensorsToUse.NONE);
            //leave time for the robot to finish turning
            sleep(250);
            //move the foundation attachment up to release the foundation
            moveFoundationServoUp();
            //leave time for the robot to finish moving the foundation
            sleep(250);

            //Strafe left toward the middle of the field
            myEncoderDrive(Direction.STRAFE_LEFT, 0.3, 6, 3, SensorsToUse.NONE);
            //move to the red tape under the bridge
            myEncoderDrive(Direction.BACKWARD, 0.4, 37, 5, SensorsToUse.NONE);
            //Strafe out of the way so we aren't in the middle of the bridge
            myEncoderDrive(Direction.STRAFE_LEFT, 0.3, 3, 5, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}
