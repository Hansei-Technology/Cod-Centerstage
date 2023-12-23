package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.teamcode.util.PIDController;

import java.util.ArrayList;

@TeleOp
@Config
public class LiftTester extends LinearOpMode {

    DcMotorEx motor;

    int rowPosition[] = { //TODO:SET THIS VALUES
            0, //base
            150, //row1
            300, //row2
            500, //row3
            700, //row4
            900, //row5
            1100, //row6
            1300, //row7
            1500, //row8
            1700, //row9
            1900, //row10
            2150, //row11
    };

    public static double kp = 0, kd = 0, ki = 0;
    PIDController pidController = new PIDController(kp, kd, ki);

    int currentPosition;
    public static int targetRow;

    ElapsedTime timer = new ElapsedTime();
    double milliseconds = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "m1");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        pidController.maxOutput = 1;

        waitForStart();

        while(opModeIsActive()) {
            currentPosition = motor.getCurrentPosition();
            if(timer.milliseconds() > milliseconds)
            {
                milliseconds = 0;
            }
            if(milliseconds == 0)
            {
                pidController.targetValue = rowPosition[targetRow];
            }

            double powerLift = pidController.update(currentPosition);
            //powerLift = Math.max(-1, Math.min(powerLift, 1));
            motor.setPower(powerLift);
        }
    }

    public void setTargetRowAfterTimer(int milliseconds, int targetRow)
    {
        timer.reset();
        this.milliseconds = milliseconds;
        this.targetRow = targetRow;

    }
}
