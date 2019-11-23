package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.CRServo;
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


        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();        //waitForStart();
            // Do not use waitForStart() if you have Motorola E4 phones.
            //waitForStart();
        }

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
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



            else
            {
                robotCallisto.stopAllMotors();
            }

        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}



