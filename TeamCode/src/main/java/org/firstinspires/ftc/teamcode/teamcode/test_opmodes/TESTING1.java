package org.firstinspires.ftc.teamcode.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp

public class TESTING1 extends LinearOpMode {
    Servo s1;
    public static double poz = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        s1=hardwareMap.get(Servo.class,"s1");
        waitForStart();
        while(opModeIsActive())
        {
            s1.setPosition(poz);
        }
    }
}
