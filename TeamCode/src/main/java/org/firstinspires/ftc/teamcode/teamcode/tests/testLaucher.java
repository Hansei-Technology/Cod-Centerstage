package org.firstinspires.ftc.teamcode.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Tests")
public class testLaucher extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx huuuuuStanga, huuuuuDreapta;

        huuuuuStanga = hardwareMap.get(DcMotorEx.class, "lansatorStanga");
        huuuuuDreapta = hardwareMap.get(DcMotorEx.class, "lansatorDreapta");

        huuuuuDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        huuuuuStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        huuuuuStanga.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.a)
            {
                huuuuuDreapta.setPower(1);
                huuuuuStanga.setPower(1);
            } else
            {
                huuuuuDreapta.setPower(0);
                huuuuuStanga.setPower(0);
            }
        }
    }
}
