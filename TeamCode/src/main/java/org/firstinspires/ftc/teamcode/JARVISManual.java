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

public class JARVISManual extends LinearOpMode {
    JARVISHW robotJARVIS = new JARVISHW();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //This is where we set our motor powers
        double motor_power = 0.7;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robotJARVIS.init(hardwareMap);

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
                //forward
                robotJARVIS.moveHolonomic(0, motor_power * 1, 0);
            }
            if (gamepad1.dpad_down) {
                //backwards
                robotJARVIS.moveHolonomic(0, motor_power * -1, 0);
            }
            if (gamepad1.dpad_right) {
                //rotate counter-clockwise
                robotJARVIS.moveHolonomic(0, 0, motor_power * 1);
            }
            if (gamepad1.dpad_left) {
                //rotate clockwise
                robotJARVIS.moveHolonomic(0, 0, motor_power * -1);

                if (gamepad1.right_bumper) {
                    //strafe right
                    robotJARVIS.moveHolonomic(motor_power * 1, 0, 0);
                }
                if (gamepad1.left_bumper) {
                    //strafe left
                    robotJARVIS.moveHolonomic(motor_power * -1, 0, 0);
                }

            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}




