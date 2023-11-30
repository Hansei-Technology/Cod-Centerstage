package org.firstinspires.ftc.teamcode.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawController {
    public Servo servoLeft, servoRight;
    public ClawController(HardwareMap map) {
        servoLeft = map.get(Servo.class, "clawLeft");
        servoRight = map.get(Servo.class, "clawRight");
    }

    public static double LEFT_CLOSE_POS = 0.56;
    public static double RIGHT_OPEN_POS = 0.255;
    public static double RIGHT_CLOSE_POS = 0.49;
    public static double LEFT_OPEN_POS = 0.8;

    public GripperStatus statusLeft = GripperStatus.CLOSED, statusRight = GripperStatus.CLOSED;
    public GripperStatus previousStatusLeft = GripperStatus.CLOSED, previousStatusRight = GripperStatus.CLOSED;

    public enum GripperStatus {
        OPEN,
        CLOSED
    }

    public void update() {
        if(statusLeft != previousStatusLeft) {
            switch (statusLeft) {
                case OPEN: {
                    servoLeft.setPosition(LEFT_OPEN_POS);
                    break;
                }
                case CLOSED: {
                    servoLeft.setPosition(LEFT_CLOSE_POS);
                    break;
                }
            }
            previousStatusLeft = statusLeft;
        }


        if(statusRight != previousStatusRight) {
            switch (statusRight) {
                case OPEN: {
                    servoRight.setPosition(RIGHT_OPEN_POS);
                    break;
                }
                case CLOSED: {
                    servoRight.setPosition(RIGHT_CLOSE_POS);
                    break;
                }
            }
            previousStatusRight = statusRight;
        }

    }
}
