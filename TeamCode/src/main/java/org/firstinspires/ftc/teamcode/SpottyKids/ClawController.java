package org.firstinspires.ftc.teamcode.SpottyKids;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawController {
    Servo cleste;

    public ClawController(HardwareMap hardwareMap)
    {
        cleste = hardwareMap.get(Servo.class, "cleste");
    }

    public enum ClawState
    {
        OPEN,
        CLOSED
    }

    ClawState currentState = ClawState.CLOSED, previosState = ClawState.CLOSED;

    public void update()
    {
        if(currentState != previosState)
        {
            switch (currentState)
            {
                case OPEN:
                    cleste.setPosition(0.5);
                case CLOSED:
                    cleste.setPosition(0.8);
            }
            previosState = currentState;
        }
    }
}
