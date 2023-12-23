package org.firstinspires.ftc.teamcode.teamcode.controllers;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.teamcode.util.PIDController;

import java.util.ArrayList;

public class ExtensionController {
    DcMotorEx motor;

    public int POZ0 = 638;
    public int HANG = 0;

    int rowPosition[] = { //TODO:SET THIS VALUES
            0, //base
            638, //row1
            200, //row2
            300, //row3
            400, //row4
            500, //row5
            600, //row6
            700, //row7
            800, //row8
            900, //row9
            1000, //row10
            1100, //row11
    }; //MAXPOS 1100

    double kp = 0.003, kd = 0.003, ki = 0;
    PIDController pidController = new PIDController(kp, kd, ki);

    public int currentPosition;
    public int targetRow;
    UniversalStates robot;
    public int targetPos = 0;
    int MAX_POS = 1100;

    public ExtensionController(HardwareMap map, UniversalStates robot) {
        motor = map.get(DcMotorEx.class, "extension");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.robot = robot;

        pidController.maxOutput = 0.6;
    }

    public void update() {
        currentPosition = motor.getCurrentPosition();
        robot.extensionPos = currentPosition;
        pidController.p=kp;
        pidController.i=ki;
        pidController.d=kd;

        targetRow = robot.currentRow;

            if(robot.state == UniversalStates.State.ARM_DOWN || robot.state == UniversalStates.State.MOVING || robot.state == UniversalStates.State.READY_TO_HANG) {
                if (targetPos > MAX_POS) targetPos = MAX_POS;
                if (targetPos < 0) targetPos = 0;

                pidController.targetValue = targetPos;
            } else {
                pidController.targetValue = rowPosition[targetRow];
            }

        double powerLift = pidController.update(currentPosition);
        //powerLift = Math.max(-1, Math.min(powerLift * 14 / robot.voltage, 1));
        motor.setPower(powerLift);
    }

    public void goToPos(int pos) {
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.6);
    }
}
