package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

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
    public static double power = 0;

    @Override
    public void runOpMode(){

        motor = this.hardwareMap.get(DcMotorEx.class, nMotor);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);


            telemetry.addData("poz ", motor.getCurrentPosition());
            telemetry.update();
        }

    }

}
