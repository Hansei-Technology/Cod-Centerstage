package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(group = "Tests")
@Config
public class TestConfigMotorServo extends LinearOpMode
{

    public DcMotorEx motor;
    public static String nMotor = "1";
    public static double power = 0;

    public Servo servo;
    public static String nServo = "1";
    public static double pozServo = 0;

    @Override
    public void runOpMode(){

        motor = this.hardwareMap.get(DcMotorEx.class, nMotor);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(nServo != "1")
        {
            servo = this.hardwareMap.get(Servo.class, nServo);
        }

        waitForStart();

        while (opModeIsActive()){

                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(power);

                if(nServo != "1")
                    servo.setPosition(pozServo);

                telemetry.addData("poz ", motor.getCurrentPosition());
                telemetry.update();
        }

    }

}
