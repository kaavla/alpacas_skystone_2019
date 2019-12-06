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
        double motor_power = 0.3;

        float leftX, leftY, rightZ;

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

            if ((gamepad1.left_stick_y != 0)  || (gamepad1.left_stick_x !=0) || (gamepad1.right_stick_x != 0)) {
                //if (gamepad1.dpad_up) {
                leftY = gamepad1.left_stick_y* -1;
                leftX = gamepad1.left_stick_x;
                rightZ = gamepad1.right_stick_x * -1;
                robotJARVIS.moveHolonomic(leftX, leftY, rightZ);
            }
            else if (gamepad1.dpad_up) {
                //forward
                robotJARVIS.moveHolonomic(0, motor_power * 1, 0);
            }
            else if (gamepad1.dpad_down) {
                //backwards
                robotJARVIS.moveHolonomic(0, motor_power * -1, 0);
            }
            else if (gamepad1.x) {
                //strafe left
                robotJARVIS.moveHolonomic(motor_power * -1, 0, 0);
            }
            else if (gamepad1.b) {
                //strafe right
                robotJARVIS.moveHolonomic(motor_power * 1, 0, 0);
            }
            else if (gamepad1.dpad_left) {
                //rotate counter-clockwise
                telemetry.addData("Status - dpad_left pressed - counter clockwise", "Run Time: " + runtime.toString());

                robotJARVIS.moveHolonomic(0, 0, motor_power * 1);
            }
            else if (gamepad1.dpad_right) {
                //rotate clockwise
                telemetry.addData("Status - dpad_right pressed - clockwise", "Run Time: " + runtime.toString());
                robotJARVIS.moveHolonomic(0, 0, motor_power * -1);
            } else if (gamepad2.dpad_up){
                robotJARVIS.slidesUp(0.3);
            }
            else if (gamepad2.dpad_down) {
                //Slide Down
                robotJARVIS.slidesDown(0.3);
            }
            else if(gamepad2.dpad_left) {
                robotJARVIS.slideIn(0.3);
            }
            else if(gamepad2.dpad_right){
                robotJARVIS.slideOut(0.3);
            }
            else if (gamepad2.x){
                robotJARVIS.claw1();
            }
            else if (gamepad2.y){
                robotJARVIS.claw2();
            }
            else if (gamepad2.b){
                robotJARVIS.claw3();
            }
            else if(gamepad2.left_bumper){
                robotJARVIS.clawTurn1();
            }
            else if(gamepad2.right_bumper){
                robotJARVIS.clawTurn2();
            }
            else if(gamepad2.left_bumper) {
                robotJARVIS.moveFoundationServoUp();
            }
            else if(gamepad2.right_bumper) {
                robotJARVIS.moveFoundationServoDown();
            }
            else
            {
                robotJARVIS.stopAllMotors();
                robotJARVIS.slide_1.setPower(0);
            }
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        }
    }






