package org.firstinspires.ftc.teamcode.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher {

    Servo servo;
    double POS_NORMAL = 0;
    double POS_LAUNCH = 0;
    public PlaneLauncher (HardwareMap map)
    {
        servo = map.get(Servo.class, "avion");
        servo.setPosition(POS_NORMAL);
    }
    public void Lauch()
    {
        servo.setPosition(POS_LAUNCH);
    }
}
