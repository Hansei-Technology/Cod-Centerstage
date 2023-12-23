package org.firstinspires.ftc.teamcode.teamcode.tests;

import org.firstinspires.ftc.teamcode.teamcode.tests.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Testing extends LinearOpMode {
    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor=hardwareMap.get(DcMotorEx.class,"m1");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                motor.setTargetPosition(2200);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            else if(gamepad1.b)
            {
                motor.setTargetPosition(10);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            else if(gamepad1.dpad_up)
            {
                motor.setTargetPosition(Math.min(motor.getCurrentPosition() + 100, 2200));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            else if(gamepad1.dpad_down)
            {
                motor.setTargetPosition(Math.max(motor.getCurrentPosition() - 100, 10));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }


            telemetry.addData("poz", motor.getCurrentPosition());
            telemetry.update();
        }

    }
}
