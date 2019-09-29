package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Callisto Crater Simple", group = "Callisto")
//@Disabled
public class CallistoAutoCraterSimple extends CallistoAutonomousBase {

    @Override
    public void runOpMode() {
        int positionGold = 2;
        RobotLog.ii("CAL", "Enter - runOpMode - CallistoAutoCraterSimple");

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initHW();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

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
        myLanderLift(Direction.ROBOT_UP, 1, 5, 5.0);

        //sleep(50);     // pause for servos to move

        telemetry.addData("Path", "Run Complete");
        telemetry.update();
        RobotLog.ii("CAL", "Exit - runOpMode - CallistoAutoCraterSimple");
    }

    private void myDetectionRun(int position,
                                double speed,
                                double timeoutS) {


        RobotLog.ii("CAL", "Enter - myDetectionRun");
        //initialized the motor encoders
        initMotorEncoders();

        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (position == 1)
            //right most side when standing near the lander lift
                {
                    //myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 12, 20.0);
                    myCollectionLiftDown(0.7, 1.0);
                //myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 7, 10.0); //PERFECT
                //myEncoderDrive(Direction.STRAFE_RIGHT,DRIVE_SPEED,16,10.0);\
                //myEncoderDrive(Direction.FORWARD,DRIVE_SPEED,16.5,10.0);
             } else if (position == 3)
            //left most side when standing near the lander lift
            {
                //myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 12, 20.0);
                myCollectionLiftDown(0.7, 1.0);
                //myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 11, 20.0);//PERFECT
                //myEncoderDrive(Direction.STRAFE_RIGHT,DRIVE_SPEED,53,20.0);
                //rotate(160, TURN_SPEED);
                //myEncoderDrive(Direction.BACKWARD,DRIVE_SPEED,16,10.0);
                //rotate(60,TURN_SPEED);
                //myEncoderDrive(Direction.BACKWARD,DRIVE_SPEED,10,10.0);
            } else // Position = 2 also default position
            //center
            {
                //myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 12, 20.0);
                myCollectionLiftDown(0.7, 1.0);
                //myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 7, 20.0);//PERFECT
                //rotate(-180, TURN_SPEED);
                //myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED,12, 20.0);
            }
        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");


    }
}

