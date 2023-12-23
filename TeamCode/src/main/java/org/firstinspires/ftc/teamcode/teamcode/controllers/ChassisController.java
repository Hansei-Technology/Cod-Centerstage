package org.firstinspires.ftc.teamcode.teamcode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChassisController {
    DcMotorEx rightRear, rightFront, leftRear, leftFront;
    public ChassisController(HardwareMap map)
    {
        rightFront = map.get(DcMotorEx.class, "rightFront");
        rightRear = map.get(DcMotorEx.class, "rightRear");
        leftFront = map.get(DcMotorEx.class, "leftFront");
        leftRear = map.get(DcMotorEx.class, "leftRear");

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Move(Gamepad g)
    {
        double y = -g.left_stick_y; // Remember, Y stick is reversed!
        double x = g.left_stick_x;
        double rx = g.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftFront.setPower((y + x + rx) / denominator);
        leftRear.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightRear.setPower((y + x - rx) / denominator);
    }

    public void StopAndResetEncoders()
    {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
