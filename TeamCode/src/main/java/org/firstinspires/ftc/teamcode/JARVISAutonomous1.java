package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@Autonomous(name="Jarvis Auto Test 1", group="JARVIS")

public class JARVISAutonomous1 extends JARVISAutonomousBase{

        JARVISHW robotJARVIS = new JARVISHW();

        @Override
        public void runOpMode() {
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
            while (!opModeIsActive() && !isStopRequested()) {
                telemetry.addData("status", "waiting for start command...");
                telemetry.update();
            }
            myDetectionRun (positionGold, DRIVE_SPEED, 40.0);
            //sleep(50);     // pause for servos to move

            //telemetry.addData("Path", "Run Complete");
            //telemetry.update();
            RobotLog.ii("CAL", "Exit - runOpMode - CallistoAutoDepotStart");

        }

        public void myDetectionRun(int position, double speed, double timeoutS)
        {

            RobotLog.ii("CAL", "Enter - myDetectionRun");

            //initialized the motor encoders
            initMotorEncoders();

            // Ensure that the op mode is still active
            if (opModeIsActive() && !isStopRequested() )
            {

                    myEncoderDrive(Direction.FORWARD, TURN_SPEED, 15, 10.0, SensorsToUse.NONE);
                    //rotate(36, TURN_SPEED);

            }
            RobotLog.ii("CAL", "Exit - myDetectionRun");
        }
    }

