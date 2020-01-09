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

public class JARVISManual extends JARVISAutonomousBase {
    @Override
    public void runOpMode() {
        //This is where we set our motor powers
        double motor_power = 0.3;

        float leftX, leftY, rightZ;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initHW();

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

            if ((gamepad1.left_stick_y != 0) || (gamepad1.left_stick_x != 0) || (gamepad1.right_stick_x != 0)) {
                //if (gamepad1.dpad_up) {
                leftY = gamepad1.left_stick_y;
                leftX = gamepad1.left_stick_x * -1;
                rightZ = gamepad1.right_stick_x * -1;
                robot.moveHolonomic(leftX, leftY, rightZ);
            } else if (gamepad1.dpad_down) {
                //forward
                robot.moveHolonomic(0, motor_power * 1, 0);
            } else if (gamepad1.dpad_up) {
                //backwards
                robot.moveHolonomic(0, motor_power * -1, 0);
            } else if (gamepad1.dpad_left) {
                //rotate counter-clockwise
                robot.moveHolonomic(0, 0, motor_power * 1);
            } else if (gamepad1.dpad_right) {
                //rotate clockwise
                robot.moveHolonomic(0, 0, motor_power * -1);
            } else if (gamepad1.x) {
                //strafe left
                //robot.moveHolonomic(-0.4, 0, 0);
                robot.openGrabberClaw(1);
            } else if (gamepad1.b) {
                //strafe right
                //robot.moveHolonomic(0.4, 0, 0);
            } else if (gamepad1.y) {
                //Nothing
                robot.setGrabberDown(1);
                robot.closeGrabberClaw(1);
            } else if (gamepad1.a) {
                //Nothing
                robot.setGrabberUp(1);

            } else if (gamepad1.right_trigger > 0.5) {
                //Move Marker Servo Down
                telemetry.addData("Status - Right Trigger pressed ", "Run Time: " + runtime.toString());
                robot.moveMarkerServoDown();
            } else if (gamepad1.left_trigger > 0.5) {
                //Move Marker Servo Up
                telemetry.addData("Status - Left Trigger pressed ", "Run Time: " + runtime.toString());
                robot.moveMarkerServoUp();
            } else if (gamepad1.left_bumper){
                robot.moveFoundationServoUp();
            } else if (gamepad1.right_bumper) {
                robot.moveFoundationServoDown();
            }else if (gamepad2.dpad_up) {
                robot.slidesUp(0.9);
                //myEncoderSlide2(Direction.SLIDE_UP, 0.9, 6, 4, SensorsToUse.NONE);

            } else if (gamepad2.dpad_down) {
                robot.slidesDown(0.9);

                // myEncoderSlide2(Direction.SLIDE_DOWN, 0.9, 6, 4, SensorsToUse.NONE);

                //robot.slidesDown(0.9);
            } else if (gamepad2.dpad_left) {
                robot.slideIn(0.7);
            } else if (gamepad2.dpad_right) {
                robot.slideOut(0.7);
            } else if (gamepad2.x) {
                robot.openClaw();  //opens the claw
            } else if (gamepad2.b) {
                robot.closeClaw(); //closes the claw
            } else if (gamepad2.y) {
                robot.rotateClawPerpendicular();  //claw perpendicular
            } else if (gamepad2.a) {
                robot.rotateClawInline(); //claw inline
            } else if (gamepad2.left_trigger > 0.5){
                robot.resetCollectionServo();
            } else if (gamepad2.right_trigger > 0.5) {
                robot.setCollectionServo();
            }
            else {
                robot.stopAllMotors();
            }
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}







