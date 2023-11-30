package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class testezCeva extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "m1");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.dpad_down)
                motor.setPower(-0.2);
            else if(gamepad1.dpad_up)
                motor.setPower(0.4);
            else
                motor.setPower(0);
            telemetry.addData("poz", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
