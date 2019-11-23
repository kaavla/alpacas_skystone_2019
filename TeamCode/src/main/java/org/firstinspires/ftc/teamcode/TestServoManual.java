package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name = "TestServo Manual", group = "Linear Opmode")

public class TestServoManual extends LinearOpMode
{
    TestServoHW robotCallisto = new TestServoHW();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //This is where we set our motor powers
        double motor_power = 0.5;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robotCallisto.init(hardwareMap);

        //waitForStart();
        // Do not use waitForStart() if you have Motorola E4 phones.
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            if (gamepad1.dpad_up) {
                robotCallisto.turntestServoforward(motor_power);
            } else if (gamepad1.dpad_down) {
                robotCallisto.turntestServobacwards(motor_power);
            }
        }
    }}