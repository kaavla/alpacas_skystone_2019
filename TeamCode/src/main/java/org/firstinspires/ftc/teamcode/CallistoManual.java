package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Callisto Manual", group = "Linear Opmode")
//@Disabled

public class CallistoManual extends LinearOpMode
{
    CallistoHW robotCallisto = new CallistoHW();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        //This is where we set our motor powers
        double motor_power = 0.7;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robotCallisto.init(hardwareMap);

        //waitForStart();
        // Do not use waitForStart() if you have Motorola E4 phones.
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            if (robotCallisto.digitalTouch.getState() == true)
            {
                //Not pressed
                telemetry.addData("Digital Touch", "Is Not Pressed");
            }
            else
            {
                //Pressed
                telemetry.addData("Digital Touch", "Is Pressed");
            }
            telemetry.update();

            if (gamepad1.dpad_up)
            {
                robotCallisto.moveForward(motor_power);
            }
            else if (gamepad1.dpad_down)
            {
                robotCallisto.moveBackwards(motor_power);
            }
            else if (gamepad1.dpad_left)
            {
                robotCallisto.turnLeft(motor_power);
            }
            else if (gamepad1.dpad_right)
            {
                robotCallisto.turnRight(motor_power);
            }
            else if (gamepad1.b)
            {
                robotCallisto.strafeRight(motor_power);
            }
            else if (gamepad1.x)
            {
                robotCallisto.strafeleft(motor_power);
            }
            else if (gamepad1.right_bumper)
            {
                robotCallisto.diagonalforwardRight(motor_power);
            }
            else if (gamepad1.left_bumper)
            {
                robotCallisto.diagonalforwardLeft(motor_power);
            }
            else if (gamepad1.left_trigger > 0.7)
            {
                robotCallisto.diagonalbackwardsLeft(motor_power);
            }
            else if (gamepad1.right_trigger > 0.7)
            {
                robotCallisto.diagonalbackwardsRight(motor_power);
            }
            else if (gamepad1.y)
            {
                robotCallisto.forwardSlow();
            }
            else if (gamepad1.a)
            {
                robotCallisto.backwardSlow();
            }
            else if (gamepad1.left_stick_button)
            {
                robotCallisto.landerliftUp(1);
                //myLanderLiftTest(0, 0.5, 5, 2);
            }
            else if (gamepad1.right_stick_button)
            {
                robotCallisto.landerliftDown(1);
                //myLanderLiftTest(1, 0.5, 5, 2);

            }
            else if (gamepad2.dpad_up)
            {
                if (robotCallisto.digitalTouch.getState() == true)
                {
                    //Not pressed
                    telemetry.addData("Digital Touch", "Is Not Pressed");
                    robotCallisto.collectionSlideIn(1);
                }
                else
                {
                    //Pressed
                    telemetry.addData("Digital Touch", "Is Pressed");
                    robotCallisto.collectionSlideOut(0);
                }


            }
            else if (gamepad2.dpad_down)
            {
                robotCallisto.collectionSlideOut(1);
            }
            else if (gamepad2.dpad_left)
            {
                robotCallisto.collectionLiftDown(0.7);
            }
            else if (gamepad2.dpad_right)
            {
                robotCallisto.collectionLiftUp(0.7);
            }
            else if (gamepad2.y)
            {
                robotCallisto.collectionDropLiftUp(0.9);
            }
            else if (gamepad2.a)
            {
                robotCallisto.collectionDropLiftDown(0.7);
            }
            else if (gamepad2.left_bumper)
            {
                robotCallisto.turnspinnerservobacwards(motor_power);
            }
            else if (gamepad2.right_bumper)
            {
                robotCallisto.turnspinnerservoforward(motor_power);
            }
            else if (gamepad2.b)
            {
                robotCallisto.turnTraytodrop();
            }
            else if (gamepad2.x)
            {
                robotCallisto.turnTraytocollect();
            }
            else if (gamepad2.left_stick_button)
            {
                robotCallisto.turnMarkerServotoInitPos();
            }
            else if (gamepad2.right_stick_button)
            {
                robotCallisto.turnMarkerServotoDrop();
            }


            else
            {
                robotCallisto.stopAllMotors();
            }

        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}



