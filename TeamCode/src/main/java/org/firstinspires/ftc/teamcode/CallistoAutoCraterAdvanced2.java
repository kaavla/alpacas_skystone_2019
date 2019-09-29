package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Callisto Auto Crater Advanced - 2", group="Callisto")
//@Disabled
public class CallistoAutoCraterAdvanced2 extends CallistoAutonomousBase {

    @Override
    public void runOpMode() {
        int positionGold = 2;
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
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        positionGold = myTFOD(2);
        myDetectionTest(positionGold, DRIVE_SPEED, 40.0);
        myDetectionRun(positionGold, DRIVE_SPEED, 40.0);
        myLanderLift(Direction.ROBOT_UP, 1, 7.5, 12.0);

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();


    }

    private void myDetectionRun(int position,
                                double speed,
                                double timeoutS) {


        //initialized the motor encoders
        initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 10, 10.0, SensorsToUse.NONE);
            rotate(76, TURN_SPEED + 0.3);
            if (position == 1)
            //gold mineral is right most side when standing near the lander lift
            {
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED + 0.3, 58, 10.0, SensorsToUse.NONE); //changed
                //gold mineral on left most side when standing near the lander lift
            } else if (position == 3) {
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED + 0.3, 25, 10.0, SensorsToUse.NONE);//PERFECT
            } else // Position = 2 also default position
            //gold mineral in center
            {
                myEncoderDrive(Direction.FORWARD, DRIVE_SPEED + 0.3, 45, 10.0, SensorsToUse.NONE);//chnaged
            }
            rotate(31, TURN_SPEED + 0.3);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED + 0.3, 12, 10.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED + 0.3, 7, 10.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED + 0.3, 39, 10.0, SensorsToUse.NONE);
            robot.markerServo.setPosition(0.3);
            sleep(1000);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED + 0.3, 55, 10.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED + 0.3, 10,   10.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED + 0.3, 15, 10.0, SensorsToUse.NONE);

        }
    }
}