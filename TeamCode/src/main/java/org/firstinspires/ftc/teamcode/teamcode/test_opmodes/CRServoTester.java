package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
@Config
public class CRServoTester extends LinearOpMode {
    CRServo servo;
    public static String nServo = "s1";
    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, nServo);

        waitForStart();

        while (opModeIsActive())
        {
            servo.setPower(power);
        }
    }
}
