package org.firstinspires.ftc.teamcode.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(group = "Tests")
public class test123 extends LinearOpMode
{

    public DcMotorEx motor;
    public static double power = 0.5;

    @Override
    public void runOpMode(){

        motor = this.hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();

        while (opModeIsActive()){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
        }

    }

}
