package org.firstinspires.ftc.teamcode.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;

@Config
public class Cleste
{
    public Servo cleste;
    public Cleste(HardwareMap map, String name)
    {

        cleste = map.get(Servo.class, name);
    }

//    public static double POZITIE_DESCHIS = 0.5;
//
//    public static double POZITIE_INCHIS = 0.7;
//
//    public boolean isClosed = true;
//
//    public enum state
//    {
//        open,
//        close
//    }
//
//    public state CurrentState = state.close;
//    state PreviousState = state.close;
//    public void update()
//    {
//        if(CurrentState != PreviousState)
//        {
//            switch (CurrentState)
//            {
//                case close:
//                    cleste.setPosition(POZITIE_INCHIS);
//                    break;
//                case open:
//                    cleste.setPosition(POZITIE_DESCHIS);
//                    break;
//            }
//            PreviousState = CurrentState;
//        }
//    }
    public static double CLOSE_POS = 0.8;
    public static double OPEN_POS = 0.5;

    public GripperStatus status = GripperStatus.CLOSE;

    public enum GripperStatus {
        OPEN,
        CLOSE
    }

    public void update() {
        switch (status) {
            case OPEN: {
                closeGripper();
                break;
            }
            case CLOSE: {
                openGripper();
                break;
            }
        }
    }

    public void closeGripper() {
        cleste.setPosition(CLOSE_POS);
        status = GripperStatus.CLOSE;
    }

    public void openGripper() {
        cleste.setPosition(OPEN_POS);
        status = GripperStatus.OPEN;
    }

    public void fullyOpenGripper() {
        cleste.setPosition(0.3);
        status = GripperStatus.OPEN;
    }
}
//2010
