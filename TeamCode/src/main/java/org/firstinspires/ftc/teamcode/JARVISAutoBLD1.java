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
        initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested() )
        {
            //move to the foundation
            myEncoderDrive(Direction.FORWARD, 0.3, 23, 3, SensorsToUse.NONE);
            //move the foundation attachment down
            moveFoundationServoDown();
            //turn the foundation counterclockwise
            rotate(90, 0.3);
            //strafe left to move the foundation into the building site
            myEncoderDrive(Direction.STRAFE_LEFT, 0.3, 15, 4, SensorsToUse.NONE);
            //move the foundation attachment up
            moveFoundationServoUp();
            //move to the foundation
            myEncoderDrive(Direction.BACKWARD, 0.3, 35, 3, SensorsToUse.NONE);
        }
        RobotLog.ii("CAL", "Exit - JARVISAutoBLD1");
    }

}
