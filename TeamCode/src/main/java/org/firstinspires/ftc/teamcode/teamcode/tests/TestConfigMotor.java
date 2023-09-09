package org.firstinspires.ftc.teamcode.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@Disabled
@TeleOp(group = "Tests")
@Config
public class TestConfigMotor extends LinearOpMode
{

    public DcMotorEx motor;
    public static String nMotor = "1";
    public static int power = 0;

    @Override
    public void runOpMode(){

        motor = this.hardwareMap.get(DcMotorEx.class, nMotor);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){

                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setTargetPosition(power);
                motor.setPower(0.3);

                telemetry.addData("poz ", motor.getCurrentPosition());
                telemetry.update();
        }

    }

}
