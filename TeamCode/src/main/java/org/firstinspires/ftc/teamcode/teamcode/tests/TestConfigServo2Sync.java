package org.firstinspires.ftc.teamcode.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(group = "Tests")
//@Config
public class TestConfigServo2Sync extends LinearOpMode
{

    public Servo servo;
    public Servo servo2;
    public static String nServo = "s1";
    public static String nServo2 = "s2";
    public static boolean position = false;

    @Override
    public void runOpMode(){

        servo = this.hardwareMap.get(Servo.class, nServo);
        servo2 = this.hardwareMap.get(Servo.class, nServo2);

        waitForStart();

        while (opModeIsActive()){

            if (position)
            {
                servo.setPosition(0.04);
                servo2.setPosition(1);
            }
            else
            {
                servo.setPosition(0.85);
                servo2.setPosition(0.13);
            }
        }

    }

}
