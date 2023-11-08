package org.firstinspires.ftc.teamcode.SpottyKids;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class JointController {
    Servo joint;

    public JointController(HardwareMap hardwareMap)
    {
        joint = hardwareMap.get(Servo.class, "joint");
        joint.setPosition(0.45);
    }

    public enum JointState
    {
        GOING_UP,
        GOING_DOWN,
        IDLE
    }

    JointState currentState = JointState.IDLE, previousState = JointState.IDLE;

    public void update()
    {
        if(currentState != previousState)
        {
            switch (currentState)
            {
                case GOING_UP:
                    joint.setPosition(joint.getPosition() - 0.01);
                    currentState = JointState.IDLE;
                case GOING_DOWN:
                    joint.setPosition(joint.getPosition() + 0.01);
                    currentState = JointState.IDLE;
            }
            previousState = currentState;
        }

    }
}
