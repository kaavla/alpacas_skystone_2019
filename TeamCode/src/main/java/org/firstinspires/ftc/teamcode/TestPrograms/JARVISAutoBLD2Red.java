package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.JARVISAutonomousBase;
import org.firstinspires.ftc.teamcode.JARVISHW;

@Autonomous(name="ZZZZZ1", group="JARVIS")

public class JARVISAutoBLD2Red extends JARVISAutonomousBase {

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
        RobotLog.ii("CAL", "Enter - JARVISAutoBLD2Red");

        //initialized the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {
            // move forward to stop dragging along the wall
            myEncoderDrive(Direction.FORWARD, 0.1, 2, 5, SensorsToUse.NONE);
            // Strafe left to be in a better position to move the foundation
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.1, 3, 5, SensorsToUse.NONE);
            // move to the foundation
            myEncoderDrive(Direction.FORWARD, 0.3, 22, 5, SensorsToUse.NONE);
            // for the final stretch of moving toward the foundation, it goes at a slower speed to minimize chance of errors
            myEncoderDrive(Direction.FORWARD, 0.1, 3, 5, SensorsToUse.NONE);
            // leave time for the foundation servos to move
            sleep(500);
            // move the foundation attachment down
            moveFoundationServoDown();
            // leave time for the foundation servos to move
            sleep(750);
            // move forward so when we turn, the foundation is in the right place
            myEncoderDrive(Direction.FORWARD, 0.1, 10, 5, SensorsToUse.NONE);

            // turn the foundation counterclockwise
            rotateUsingOneSide(-90, 0.2);
            // leave time for the robot to finish turning
            sleep(500);
            // push the foundation into the wall
            myEncoderDrive(Direction.FORWARD, 0.1, 15, 5, SensorsToUse.NONE);
            // leave time for the foundation servos to move
            sleep(500);
            // move the foundation attachment up to release the foundation
            moveFoundationServoUp();
            // leave time for the foundation servos to move
            sleep(500);
            // Move closer to the wall so we are out of the way of the other robot in the middle.
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.2, 10, 5, SensorsToUse.NONE);
            // strafe left to be as close to the wall as possible to stay out of the way for the other robot
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.1, 12, 5, SensorsToUse.NONE);
            // move backwards to the blue tape under the bridge
            myEncoderDrive(Direction.BACKWARD, 0.3, 42, 5, SensorsToUse.NONE);
            // strafe left to be as close to the wall as possible to stay out of the way for the other robot
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.1, 5, 5, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - JARVISAutoBLD2Red");
    }

}