package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;
@Autonomous(name="Jarvis Auto Red", group="JARVIS")

public class JARVSAutoRed extends JARVISAutonomousBase {
    static final int SIDE = 1; //Right side

    @Override
    public void runOpMode() {
        RobotLog.ii("CAL", "Enter  - runOpMode - JARVIS Autonomous 1");

        initHW();

        ref_angle = getAngle();
        //telemetry.addData("status", "ref_angle = %f", ref_angle);
        //telemetry.update();

        // Send telemetry message to signifyrobot waiting;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        myDetectionRun(40.0);

        RobotLog.ii("CAL", "Exit - runOpMode - JARVIS Autonomous 1");

    }

    public void correctAngle()
    {
        double currentAngle = 0;
        currentAngle = getAngle();
        rotate((int)((-1)*(currentAngle - ref_angle)), 0.2);
        sleep(300);
    }

    public void getStone()
    {
        myEncoderDrive(Direction.STRAFE_LEFT, 0.2, 4, 5.0, SensorsToUse.NONE);

        robot.openGrabberClaw(SIDE);
        sleep(400);

        robot.setGrabberHalfDown(SIDE);
        sleep(400);

        myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,4, 5.0, SensorsToUse.NONE);
        sleep(300);

        robot.setGrabberDown(SIDE);
        sleep(500);

        robot.closeGrabberClaw(SIDE);
        sleep(500);

        robot.setGrabberUp(SIDE);
        sleep(500);

        myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
        sleep(250);

    }

    public void releaseStone()
    {
        robot.setGrabberDown(SIDE);
        sleep(200);
        robot.openGrabberClaw(SIDE);
        sleep(200);
        robot.setGrabberUp(SIDE);
        sleep(200);
        robot.closeGrabberClaw(SIDE);
        sleep(200);

    }

    public void myDetectionRun(double timeoutS)
    {

        RobotLog.ii("CAL", "Enter - myDetectionRun");
        double strafe_back = 0;

        //initialized the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {
            //MOve towards the skystones
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.2, 50, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);
            correctAngle();

            if (myDetectSkystone(SideToUse.USE_RIGHT, 10) == false) {
                //detected stone. Strafe left to test the next one.
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 7, 5.0, SensorsToUse.NONE);

                strafe_back = strafe_back + 7;
                if (myDetectSkystone(SideToUse.USE_RIGHT, 10) == false) {
                    //detected stone. Strafe left to test the next one.
                    myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 7, 5.0, SensorsToUse.NONE);
                    strafe_back = strafe_back + 7;
                }
            }

            //Grab the skystone
            getStone();

            //Drive to the other side
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 35 + strafe_back, 10.0, SensorsToUse.NONE);
            correctAngle();

            releaseStone();

            //Drive back to collect the second stone
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 62 + strafe_back, 10.0, SensorsToUse.NONE);
            //correctAngle();

            //Drive till we are close to the stone again
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.2,20, 5.0, SensorsToUse.USE_DISTANCE_RIGHT);

            //Grab the skystone
            getStone();

            //drive to other side
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 65 + strafe_back, 10.0, SensorsToUse.NONE);
            //correctAngle();

            releaseStone();

            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 16, 10.0, SensorsToUse.NONE);
            correctAngle();
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}
