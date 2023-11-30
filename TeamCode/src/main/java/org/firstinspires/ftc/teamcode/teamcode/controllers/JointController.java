package org.firstinspires.ftc.teamcode.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class JointController {
    Servo servoLeft, servoRight;

    double POS0 = 0.67;

    double AUTO_POS=0.67;

    public int targetRow = -1, lastRow = -1;

    UniversalStates robot;

    double angleCoeff = 0;

    double rowPosition[] = { //TODO:SET THIS VALUES
            0.67, //base
            0.68, //row1
            0.68, //row2
            0.68, //row3
            0.67, //row4
            0.66, //row5
            0.65, //row6
            0.64, //row7
            0.64, //row8
            0.63, //row9
            0.63, //row10
            0.63, //row11
    };

    public JointController(HardwareMap map, UniversalStates robot) {
        servoLeft = map.get(Servo.class, "jointLeft");
        servoRight = map.get(Servo.class, "jointRight");
        this.robot = robot;
    }

    public void update() {
        targetRow = robot.currentRow;
        if(robot.state== UniversalStates.State.AUTO){
            servoLeft.setPosition(AUTO_POS);
            servoRight.setPosition(AUTO_POS);
        }
        else if(robot.state == UniversalStates.State.ARM_DOWN) {
            servoLeft.setPosition(POS0 + robot.extensionPos * angleCoeff);
            servoRight.setPosition(POS0 + robot.extensionPos * angleCoeff);
        } else if(targetRow != lastRow) {
            servoRight.setPosition(rowPosition[targetRow]);
            servoLeft.setPosition(rowPosition[targetRow]);
        }
        lastRow = targetRow;
    }
}
