package org.firstinspires.ftc.teamcode.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class lift extends TwoMotorSystem{
    public enum liftState {
        IDLE,
        GOING_UP,
        GOING_DOWN,
        HOLD_POS,
    }

    public enum Pula {
        LASATA,
        SCULATA
    }

    public liftState currentState = liftState.IDLE;
    public liftState previousState = liftState.IDLE;
    public Pula PULA = Pula.LASATA;
    public double LIMITA_PULA_JOS = 100;

    public double liftCurrentPos = 0;

    public lift(HardwareMap map, String motorLeft, String motorRight)
    {
        super(map, motorLeft, motorRight);
        this.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update()
    {
        liftCurrentPos = getCurrentPosition();
        if(currentState!=previousState) {
            switch (currentState)
            {
                case GOING_DOWN: {
                    setPower(-0.5);
                    break;
                }
                case GOING_UP: {
                    setPower(0.5);
                    break;
                }
                case HOLD_POS: {
                    setPower(0);
                    break;
                }
            }
            if(getCurrentPosition() > LIMITA_PULA_JOS) {
                PULA = Pula.SCULATA;
            } else {
                PULA = Pula.LASATA;
            }
            previousState = currentState;
        }

    }
}
