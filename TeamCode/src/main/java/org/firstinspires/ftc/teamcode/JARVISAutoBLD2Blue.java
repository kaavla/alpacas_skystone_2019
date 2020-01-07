package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Jarvis Auto Build Site 2 Blue", group="JARVIS")

public class JARVISAutoBLD2Blue extends JARVISAutonomousBase {

    JARVISHW robotJARVIS = new JARVISHW();

    @Override
    public void runOpMode() {
        // Initializes the motors so they are ready for use.
        robot.init(hardwareMap);
        // move the foundation attachment up to the start position
        moveFoundationServoUp();

        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();


        // Send telemetry message to signify robot waiting;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        // Runs the program
        autoBLDFoundation();
    }

    public void autoBLDFoundation()
    {
        RobotLog.ii("CAL", "Enter - JARVISAutoBLD2Blue");

        //initialized the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {
            //move forward to stop dragging along the wall
            myEncoderDrive(Direction.BACKWARD, 0.1, 2, 5, SensorsToUse.NONE);
            //Strafe left to be in a better position to move the foundation
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.1, 8, 5, SensorsToUse.NONE);
            //move to the foundation
            myEncoderDrive(Direction.BACKWARD, 0.3, 18, 5, SensorsToUse.NONE);
            //go at a slower speed to make sure we hook on properly
            myEncoderDrive(Direction.BACKWARD, 0.1, 7, 5, SensorsToUse.NONE);
            //leave time for the foundation servos to move
            sleep(500);
            //move the foundation attachment down
            moveFoundationServoDown();
            //leave time for the foundation servos to move
            sleep(500);

            //move diagonally to bring the foundation away from the wall
            myEncoderDrive(Direction.DIAGONAL_LEFT, 0.2, 10, 5, SensorsToUse.NONE);
            //turn the foundation so it is parallel to the front wall
            rotate(90, 0.4);
            //move backwards with the foundation and bring it close to the wall
            myEncoderDrive(Direction.BACKWARD, 0.2, 3, 5, SensorsToUse.NONE);
            //leave time for the robot to finish turning
            sleep(500);
            //move the foundation attachment up to release the foundation
            moveFoundationServoUp();
            //strafe to the side to get out of the way of the other robot while waiting
            myEncoderDrive(Direction.DIAGONAL_RIGHT, 0.3, 4, 5, SensorsToUse.NONE);
            //stop for 10 seconds so the robot is out of the way of the other robots while the
            //autonomous mode is still going on
            sleep(10000);
            //move to the blue tape under the bridge
            myEncoderDrive(Direction.FORWARD, 0.3, 35, 5, SensorsToUse.NONE);
            //strafe into the wall
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.1, 5, 5, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - JARVISAutoBLD2Blue");
    }

}