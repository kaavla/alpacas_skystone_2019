package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.Direction.ROBOT_DOWN;

@Autonomous(name="Callisto Depot Start", group="Callisto")
//@Disabled
public class CallistoAutoDepotStart extends CallistoAutonomousBase
{
    CallistoHW robotCallisto = new CallistoHW();

    @Override
    public void runOpMode()
    {
        int positionGold = 2;
        RobotLog.ii("CAL", "Enter - runOpMode - CallistoAutoDepotStart");
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initHW();

        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();


        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Status", "Resetting Encoders");    //
        //telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        // Do not use waitForStart() if you have Motorola E4 phones.
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        positionGold = myTFOD(2);
        myDetectionTest(positionGold, DRIVE_SPEED, 40.0);
        myDetectionRun(positionGold, DRIVE_SPEED, 40.0);
        myLanderLift(Direction.ROBOT_UP, 1, 3, 2.0);

        //sleep(50);     // pause for servos to move

        //telemetry.addData("Path", "Run Complete");
        //telemetry.update();
        RobotLog.ii("CAL", "Exit - runOpMode - CallistoAutoDepotStart");

    }

    private void myDetectionRun(int position,
                                double speed,
                                double timeoutS)
    {

        RobotLog.ii("CAL", "Enter - myDetectionRun");

        //initialized the motor encoders
        initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {

            // Determine new target position, and pass to motor controller
            if (position == 1)
            {
                //cube on robot right - human left
                myEncoderDrive(Direction.FORWARD, TURN_SPEED, 15, 10.0, SensorsToUse.NONE);
                rotate(36, TURN_SPEED);
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 19, 10.0, SensorsToUse.NONE);
                robot.markerServo.setPosition(0.3);
                sleep(1000);
                rotate(82, TURN_SPEED);
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 10, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 11, 10.0, SensorsToUse.NONE);
//                myEncoderDrive(Direction.FORWARD, 0.95, 52, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.FORWARD, 0.95, 30, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.STRAFE_LEFT, TURN_SPEED, 6, 5.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.FORWARD, 0.95, 22, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.STRAFE_RIGHT, TURN_SPEED, 9, 5.0, SensorsToUse.NONE);
            }
             else if (position == 3)
            {
                //cube on robot left - human right
                myEncoderDrive(Direction.FORWARD, TURN_SPEED, 25, 10.0, SensorsToUse.NONE);
                robot.markerServo.setPosition(0.3);
                sleep(1000);
                myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 3, 5.0, SensorsToUse.NONE);
                rotate(122, TURN_SPEED);
                //myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 12, 10.0);
                myEncoderDrive(Direction.FORWARD, 0.95, 48, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 12, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.FORWARD, 0.95, 10, 10.0, SensorsToUse.NONE);
            } else // Position = 2 also default position
            {
                //cube in middle
                //myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 4, 5.0);
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 28, 10.0, SensorsToUse.NONE);
                robot.markerServo.setPosition(0.3);
                sleep(1000);
                rotate(113, TURN_SPEED);
                myEncoderDrive(Direction.FORWARD, 0.95, 15, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED, 3, 5.0, SensorsToUse.NONE);
                //myEncoderDrive(Direction.FORWARD, 0.95, 54, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.FORWARD, 0.95, 30, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.STRAFE_LEFT, TURN_SPEED, 4, 5.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.FORWARD, 0.95, 24, 10.0, SensorsToUse.NONE);
                myEncoderDrive(Direction.STRAFE_RIGHT, TURN_SPEED, 9, 5.0, SensorsToUse.NONE);
            }


            myCollectionLiftDown(0.7, 1.0);
            myCollectionSlideOut(0.8,1);
            robot.turnspinnerservobacwards(1);
        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}
