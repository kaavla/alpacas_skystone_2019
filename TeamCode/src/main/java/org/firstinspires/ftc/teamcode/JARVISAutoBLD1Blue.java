package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Jarvis Auto Build Site 1 Blue", group="JARVIS")

public class JARVISAutoBLD1Blue extends JARVISAutonomousBase {

    JARVISHW robotJARVIS = new JARVISHW();

    @Override
    public void runOpMode() {
        // Initializes all of the hardware so it can be used in the code.
        robot.init(hardwareMap);
        // Move the foundation attachment up to the start position
        moveFoundationServoUp();

        ref_angle = getAngle();
        telemetry.addData("status", "ref_angle = %f", ref_angle);
        telemetry.update();


        // Send telemetry message to signify robot is ready to run;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        // Run the program
        autoBLDFoundation();
    }

    public void autoBLDFoundation()
    {
        RobotLog.ii("CAL", "Enter - JARVISAutoBLD1Blue");

        //initialized the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {
            //move forward to stop dragging along the wall
            myEncoderDrive(Direction.FORWARD, 0.1, 2, 5, SensorsToUse.NONE);
            //Strafe left to be in a better position to move the foundation
            myEncoderDrive(Direction.STRAFE_LEFT, 0.1, 8, 5, SensorsToUse.NONE);
            //move to the foundation
            myEncoderDrive(Direction.FORWARD, 0.3, 22, 5, SensorsToUse.NONE);
            //leave time for the foundation servos to move
            sleep(500);
            //move the foundation attachment down
            moveFoundationServoDown();
            //leave time for the foundation servos to move
            sleep(500);


            //move backwards with the foundation and bring it close to the wall
            myEncoderDrive(Direction.BACKWARD, 0.2, 30, 5, SensorsToUse.NONE);
            //leave time for the robot to finish turning
            sleep(500);
            //move the foundation attachment up to release the foundation
            moveFoundationServoUp();
            //leave time for the foundation servos to move
            sleep(500);
            //move to the blue tape under the bridge
            myEncoderDrive(Direction.STRAFE_RIGHT, 0.3, 42, 5, SensorsToUse.NONE);
            //Turn the robot so it is parallel to the wall
            rotate(90, 0.1);
            //move the robot into the wall to stay out of the way of the other robot.
            myEncoderDrive(Direction.STRAFE_LEFT, 0.1, 3, 5, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - JARVISAutoBLD1Blue");
    }

}