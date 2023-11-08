package org.firstinspires.ftc.teamcode.SpottyKids;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmController {
    DcMotorEx brat;

    public ArmController(HardwareMap hardwareMap)
    {
        brat = hardwareMap.get(DcMotorEx.class, "brat");
    }

    public enum ArmState
    {
        UP,
        DOWN
    }

    ArmState currentState = ArmState.DOWN, previousState = ArmState.DOWN;

    public void update()
    {
        if(currentState != previousState)
        {
            switch (currentState)
            {
                case UP:
                    brat.setTargetPosition(120);
                    brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    brat.setPower(0.7);
                case DOWN:
                    brat.setTargetPosition(15);
                    brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    brat.setPower(0.7);
            }
            previousState = currentState;
        }
    }

}
