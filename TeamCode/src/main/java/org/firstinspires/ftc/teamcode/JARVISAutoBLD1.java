package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Jarvis Auto Build Site 1", group="JARVIS")

public class JARVISAutoBLD1 extends JARVISAutonomousBase {

    JARVISHW robotJARVIS = new JARVISHW();

    @Override
    public void runOpMode() {
        initHW();

        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();


        // Send telemetry message to signify robot waiting;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        myDetectionRun (0, 0.3, 40.0);
        //sleep(50);     // pause for servos to move
    }

    public void myDetectionRun(int position, double speed, double timeoutS)
    {

        RobotLog.ii("CAL", "Enter - JARVISAutoBLD1");

        //initialized the motor encoders
        robot.initMotorEncoders();
        //move the foundation attachment up to the start position
        //moveFoundationServoUp();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {
            //move to the foundation
            myEncoderDrive(Direction.FORWARD, 0.2, 24, 5, SensorsToUse.NONE);
            //for the final stretch of moving toward the foundation, it goes at a slower speed to minimize chance of errors
            myEncoderDrive(Direction.FORWARD, 0.1, 4, 5, SensorsToUse.NONE);
            //move the foundation attachment down
            //moveFoundationServoDown();
            //turn the foundation counterclockwise
            rotate(86, 0.1);
            //wait for the servos to stop moving
            sleep(1000);
            //strafe left to move the foundation into the building site
            myEncoderDrive(Direction.STRAFE_LEFT, 0.15, 32, 5, SensorsToUse.NONE);
            //move the foundation attachment up
            //moveFoundationServoUp();
            //back away from the foundation to leave space to strafe
            myEncoderDrive(Direction.BACKWARD, 0.1, 1, 5, SensorsToUse.NONE);
            //strafe away from the wall
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.1, 1, 5, SensorsToUse.NONE);
            //move to the foundation
            myEncoderDrive(Direction.BACKWARD, 0.2, 41, 5, SensorsToUse.NONE);
            //strafe left to move the robot up against the wall so it is out of the way of the other robot
            myEncoderDrive(Direction.STRAFE_LEFT, 0.17, 3, 5, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - JARVISAutoBLD1");
    }

}