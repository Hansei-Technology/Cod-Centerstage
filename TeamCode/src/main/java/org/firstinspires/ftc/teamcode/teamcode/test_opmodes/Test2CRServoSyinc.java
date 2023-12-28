package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Config
public class Test2CRServoSyinc extends LinearOpMode {
    CRServo servo1, servo2;
    public static String nServo1 = "s1", nServo2 = "s2";
    public static double power1 = 0, power2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(CRServo.class, nServo1);
        servo1 = hardwareMap.get(CRServo.class, nServo2);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.a) {
                servo1.setPower(power1);
                servo2.setPower(power2);
            } else {
                servo1.setPower(0);
                servo2.setPower(0);
            }
        }
    }
}
