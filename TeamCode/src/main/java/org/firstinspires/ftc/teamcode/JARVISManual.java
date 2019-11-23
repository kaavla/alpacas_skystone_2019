package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "JARVIS Manual", group = "Linear Opmode")
//@Disabled

public class JARVISManual extends LinearOpMode{
    JARVISHW robotJARVIS = new JARVISHW();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //This is where we set our motor powers
        double motor_power = 0.7;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robotJARVIS.init(hardwareMap);
        //we make sure to update the phones with the data occuring by using telemetry
        //waitForStart();
        // Do not use waitForStart() if you have Motorola E4 phones.

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //all of the code bellow is setting a power to each button on the gamepad
            if (gamepad1.dpad_up) {
                robotJARVIS.moveForward(motor_power);
            } else if (gamepad1.dpad_down) {
                robotJARVIS.moveBackwards(motor_power);
            } else if (gamepad1.dpad_left) {
                robotJARVIS.turnLeft(motor_power);
            } else if (gamepad1.dpad_right) {
                robotJARVIS.turnRight(motor_power);
            } else if (gamepad1.b) {
                robotJARVIS.strafeRight(motor_power);
            } else if (gamepad1.x) {
                robotJARVIS.strafeleft(motor_power);
            } else if (gamepad1.right_bumper) {
                robotJARVIS.diagonalforwardRight(motor_power);
            } else if (gamepad1.left_bumper) {
                robotJARVIS.diagonalforwardLeft(motor_power);
            } else if (gamepad1.left_trigger > 0.7) {
                robotJARVIS.diagonalbackwardsLeft(motor_power);
            } else if (gamepad1.right_trigger > 0.7) {
                robotJARVIS.diagonalbackwardsRight(motor_power);
            } else if (gamepad1.y) {
                robotJARVIS.forwardSlow();
            } else if (gamepad1.a) {
                robotJARVIS.backwardSlow();
            } else if (gamepad1.right_trigger > 0.7) {

            } else if (gamepad1.right_stick_x > -1.0) {
                robotJARVIS.moveHolonomic(0, 0);
            }
            else if (gamepad1.right_stick_y > -1.0) {
                robotJARVIS.moveHolonomic(0, 0);
            }
            else if (gamepad1.left_stick_x > -1.0) {
                robotJARVIS.moveHolonomic(0, 0);
            }
            else if (gamepad1.left_stick_y > -1.0) {
                robotJARVIS.moveHolonomic(0, 0);
            }

                else {
                    robotJARVIS.stopAllMotors();
                }

                }
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }




