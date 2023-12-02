package org.firstinspires.ftc.teamcode.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.teamcode.util.PIDController;

import java.util.ArrayList;

@Config
public class ArmController {
    public DcMotorEx motor;
    public int MOVING_POS = 400;
    public int DOWN_POS = 30;
    public int AUTO_POS = 3200;

    public int HANG = 1900;

    public int rowPosition[] = { //TODO:SET THIS VALUES
            4700-1500, //base
            3000, //row1
            4650-1500, //row2
            4600-1500, //row3
            4550-1500, //row4
            4500-1500, //row5
            4450-1500, //row6
            4400-1500, //row7
            4350-1500, //row8
            4300-1500, //row9
            4250-1500, //row10
            4200-1500, //row11
    };

    public static double kp = 0.02, kd = 0.03, ki = 0.01;
    PIDController pidController = new PIDController(kp, kd, ki);

    public int currentPosition;
    public int targetRow;
    public UniversalStates robot;

    public ArmController(HardwareMap map, UniversalStates robot) {
        motor = map.get(DcMotorEx.class, "armMotor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.robot = robot;

        pidController.maxOutput = 1;
        pidController.p=kp;
        pidController.i=ki;
        pidController.d=kd;
    }

    public void update() {
        pidController.p = kp;
        pidController.i = ki;
        pidController.d = kd;

        currentPosition = motor.getCurrentPosition();
        switch (robot.state) {
                case ARM_DOWN: {
                    pidController.maxOutput = 0.5;
                    pidController.targetValue = DOWN_POS;
                    break;
                }
                case MOVING: {
                    pidController.maxOutput = 1;
                    pidController.targetValue = MOVING_POS;
                    break;
                }
            case AUTO: {
                pidController.maxOutput = 1;
                pidController.targetValue = AUTO_POS;
                break;
            }
            case ARM_UP: {
                pidController.maxOutput = 1;
                pidController.targetValue = rowPosition[robot.currentRow];
                break;
            }
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
