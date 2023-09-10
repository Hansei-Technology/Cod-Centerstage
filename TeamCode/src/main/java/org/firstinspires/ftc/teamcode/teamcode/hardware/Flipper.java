package org.firstinspires.ftc.teamcode.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@Config
public class Flipper {

//    public class FlipperPos {
//        public double POS_STANGA, POS_DREAPTA;
//        public FlipperPos(double pos_s, double pos_d) {
//            this.POS_STANGA = pos_s;
//            this.POS_DREAPTA = pos_d;
//        }
//    }

    public static double POS_IDLE = 0;
    public static double POS_DROP = 0.7;

    public enum FlipperState {
        IDLE,
        DROP,
    }

    public FlipperState currentState = FlipperState.IDLE, previousState = FlipperState.IDLE;
    public Servo servo;

    public Flipper(HardwareMap hardwareMap, String numeServo) {
        servo = hardwareMap.get(Servo.class, numeServo);
    }


    public void update() {
        if(currentState!=currentState) {
            switch (currentState) {
                case IDLE:
                    servo.setPosition(POS_IDLE);
                    break;
                case DROP:
                    servo.setPosition(POS_DROP);
                    break;
            }
            previousState = currentState;
        }
    }
}
