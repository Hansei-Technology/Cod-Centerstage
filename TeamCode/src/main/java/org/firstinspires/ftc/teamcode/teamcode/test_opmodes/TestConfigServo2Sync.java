package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Tests")
@Config
public class TestConfigServo2Sync extends LinearOpMode
{

    public Servo servo;
    public Servo servo2;
    public static String nServo = "s1";
    public static String nServo2 = "s2";
    public static double position = 0;

    @Override
    public void runOpMode(){

        servo = this.hardwareMap.get(Servo.class, nServo);
        servo2 = this.hardwareMap.get(Servo.class, nServo2);

        waitForStart();

        while (opModeIsActive()){
            servo.setPosition(position);
            servo2.setPosition(position);
        }

    }

}
