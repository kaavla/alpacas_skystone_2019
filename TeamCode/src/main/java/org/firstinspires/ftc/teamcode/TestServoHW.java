package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

public class TestServoHW
{
    public CRServo testServo = null;

    public void init(HardwareMap ahwMap) {
        RobotLog.ii("CAL", "Enter - init");
        testServo = ahwMap.get(CRServo.class, "testServo");
    }
    public void stopAllMotors()
    {
        testServo.setPower(0);
    }
    public void turntestServoforward(double power)
    {
        testServo.setPower(power);
    }

    public void turntestServobacwards(double power) {
        testServo.setPower(-1 * power);
    }
}
