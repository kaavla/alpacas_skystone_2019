package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Jarvis Auto Build Site 1 Blue", group="JARVIS")

public class JARVISAutoBLD1Red extends JARVISAutonomousBase {

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

        RobotLog.ii("CAL", "Enter - JARVISAutoBLD1Red");

        //initialized the motor encoders
        initMotorEncoders();
        //move the foundation attachment up to the start position
        robotJARVIS.moveFoundationServoUp();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {
            //move to the foundation
            myEncoderDrive(Direction.FORWARD, 0.3, 24, 5, SensorsToUse.NONE);
            //for the final stretch of moving toward the foundation, it goes at a slower speed to minimize chance of errors
            myEncoderDrive(Direction.FORWARD, 0.1, 4, 5, SensorsToUse.NONE);
            //move the foundation attachment down
            robotJARVIS.moveFoundationServoDown();

            //move backwards with the foundation and bring it close to the wall
            myEncoderDrive(Direction.BACKWARD, 0.2, 26, 5, SensorsToUse.NONE);
            //turn the foundation counterclockwise
            rotate(-86, 0.1);
            //leave time for the robot to finish turning
            sleep(500);
            //push the foundation into the wall
            myEncoderDrive(Direction.FORWARD, 0.1, 2.5, 5, SensorsToUse.NONE);
            //move the foundation attachment up to release the foundation
            robotJARVIS.moveFoundationServoUp();
            //move backwards to the blue tape under the bridge
            myEncoderDrive(Direction.BACKWARD, 0.3, 41, 5, SensorsToUse.NONE);
            //strafe left to be as close to the wall as possible to stay out of the way for the other robot
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.1, 5, 5, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - JARVISAutoBLD1Red");
    }

}