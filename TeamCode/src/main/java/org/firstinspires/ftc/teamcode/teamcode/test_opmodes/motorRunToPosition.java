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
public class motorRunToPosition extends LinearOpMode {

    DcMotorEx motor;
    public static double kp = 0, kd = 0, ki = 0;
    PIDController pidController = new PIDController(kp, kd, ki);

    int currentPosition;
    public static int target;

    public static String nume = "";

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, nume);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController.maxOutput = 1;

        waitForStart();

        while(opModeIsActive()) {
            pidController.p = kp;
            pidController.i = ki;
            pidController.d = kd;

            currentPosition = motor.getCurrentPosition();
            pidController.targetValue = target;

            double powerLift = pidController.update(currentPosition);
            //powerLift = Math.max(-1, Math.min(powerLift, 1));
            motor.setPower(powerLift);
        }
    }
}
