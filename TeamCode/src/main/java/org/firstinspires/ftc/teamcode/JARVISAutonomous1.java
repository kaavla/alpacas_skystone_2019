package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;
@Autonomous(name="Jarvis Auto Test 3", group="JARVIS")

public class JARVISAutonomous1 extends JARVISAutonomousBase {


    @Override
    public void runOpMode() {
        //int positionGold = 2;
        initHW();

        ref_angle = getAngle();
        //telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();

        // Send telemetry message to signifyrobot waiting;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        //myDetectionRun (positionGold, 0.3, 40.0);
        //sleep(50);     // pause for servos to move

        //telemetry.addData("Path", "Run Complete");
        //telemetry.update();
        myDetectionRun(40.0);
        RobotLog.ii("CAL", "Enter - runOpMode - JARVIS Autonomous 1");
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        RobotLog.ii("CAL", "Exit - runOpMode - JARVIS Autonomous 1");

    }

    public void myDetectionRun(double timeoutS)
    {

        RobotLog.ii("CAL", "Enter - myDetectionRun");

        robot.initMotorNoEncoders();
        //myTFOD(30.0);

        //initialized the motor encoders
        robot.initMotorEncoders();

        RobotLog.ii("CAL", "initMotorEncoders-Done");


        RobotLog.ii("CAL", "myTFOD Done");

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            //MOve forward
            /*if (myTFOD2(10) == false) {
                //strafe right some distance
                myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
                if (myTFOD2(10) == false) {
                    myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
                    //strafe right
                }
            }

             */

//after these too we assume that the skystone is the third and it will play out the code below
            robot.claw1();
            sleep(1000);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 30, 10,SensorsToUse.NONE);
            mySlideAuto(-0.3, 1);
            sleep(1000);
            robot.clawTurn1();
            sleep(1000);
            mySlidesAuto(0.3, 4.0);
            sleep(1000);
            robot.claw3();
            sleep(1000);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 52, 15.0, SensorsToUse.NONE);
            robot.claw1();
            sleep(1000);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 75, 15.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 5, 5.0, SensorsToUse.NONE);
            mySlidesAuto(-0.3, 5.0);
            sleep(1000);
            robot.claw3();
            sleep(1000);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 6, 5.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED, 72, 10.0, SensorsToUse.NONE);
            robot.claw1();
            sleep(1000);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 19, 10.0, SensorsToUse.NONE);

        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}
