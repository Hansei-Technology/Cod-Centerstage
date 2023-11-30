package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Tests")
@Config
public class TestConfigServo2 extends LinearOpMode
{
    public Servo servo;
    public Servo servo2;
    public static String nServo = "s1";
    public static String nServo2 = "s2";

    public static double power = 0;
    public static double power2 = 0;


    @Override
    public void runOpMode(){

        servo = this.hardwareMap.get(Servo.class, nServo);
        servo2 = this.hardwareMap.get(Servo.class, nServo2);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()){
                servo.setPosition(power);
                servo2.setPosition(power2);
        }

    }

}
