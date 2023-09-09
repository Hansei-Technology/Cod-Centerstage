package org.firstinspires.ftc.teamcode.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(group = "Tests")
@Config
public class TestConfigServo extends LinearOpMode
{

    public Servo servo;
    public static String nServo = "sleeveS";
    public static double power = 0;

    @Override
    public void runOpMode(){

        servo = this.hardwareMap.get(Servo.class, nServo);

        waitForStart();

        while (opModeIsActive()){

            servo.setPosition(power);
        }

    }

}
