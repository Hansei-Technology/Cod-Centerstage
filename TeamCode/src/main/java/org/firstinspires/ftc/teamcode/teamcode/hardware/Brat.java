package org.firstinspires.ftc.teamcode.teamcode.hardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Brat {

    public DcMotorEx motor;
    public Servo joint;

    public class PosBrat {
        public double POS_SERVO;
        public int POS_MOTOR;
        public PosBrat(int pos_motor, double pos_servo) {
            this.POS_MOTOR = pos_motor;
            this.POS_SERVO = pos_servo;
        }
    }

    public enum JointStatus {
        GRAB,
        IDLE,
        INIT,
        AUTO,
        INIT_AUTO
    }

    public JointStatus jointStatus = JointStatus.INIT;
    double powSus = 0.8;
    double powJos = -0.1;

    public PosBrat posStorage = new PosBrat(120, 0.525);
    public PosBrat posBoard = new PosBrat(120, 0.47);
    public PosBrat posGround = new PosBrat(0, 0.5);
    public PosBrat posGround2 = new PosBrat(15, 0.5);
    public PosBrat posJointIdle = new PosBrat(0, 0.48);
    public PosBrat posGrab = new PosBrat(0, 0.5);

    public Brat(HardwareMap map, String numeMotor, String numeJoint) throws InterruptedException {
        motor = map.get(DcMotorEx.class, numeMotor);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint = map.get(Servo.class, numeJoint);
        joint.setPosition(0.5);
    }
    //brat init 115
    public enum state
    {
        bratGoingUp,
        bratGoingDown,
        jointGoingUp,
        jointGoingDown,
        stay,
        goToStorage,
        goToGround,
        goToBoard,
        gotoGrab,
        noUpdate,
        init
    }

    public state CurrentState = state.stay;
    state PreviousState = state.stay;

    public void update() throws InterruptedException {
        if(CurrentState != PreviousState)
        {
            switch(CurrentState)
            {
                case gotoGrab:
                    motor.setTargetPosition(posGrab.POS_MOTOR);
                    motor.setPower(0.8);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    joint.setPosition(posGrab.POS_SERVO);
                case bratGoingUp:
                    motor.setPower(powSus);
                    break;
                case bratGoingDown:
                    motor.setPower(powJos);
                    break;
                case jointGoingUp:
                    joint.setPosition(joint.getPosition() + 0.01);
                    break;
                case jointGoingDown:
                    joint.setPosition(joint.getPosition() - 0.01);
                    break;
                case stay:
                    motor.setPower(0);
                    break;
                case goToBoard:
                    motor.setTargetPosition(posBoard.POS_MOTOR);
                    motor.setPower(0.2);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    joint.setPosition(posBoard.POS_SERVO);
                    break;
                case goToGround:
                    joint.setPosition(posGround.POS_SERVO);
                    jointStatus = JointStatus.GRAB;
                    sleep(200);
                    motor.setTargetPosition(posGround2.POS_MOTOR);
                    motor.setPower(0.4);
                    sleep(700);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setTargetPosition(posGround.POS_MOTOR);
                    motor.setPower(0.3);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                case goToStorage:
                    motor.setTargetPosition(posStorage.POS_MOTOR);
                    motor.setPower(0.8);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    joint.setPosition(posStorage.POS_SERVO);
                    break;
            }
            PreviousState = CurrentState;
        }
    }


    public void moveToInit() throws InterruptedException {
        motor.setTargetPosition(135);
        motor.setPower(0.4);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(800);
        joint.setPosition(0.55);
    }

    public void moveToGame() throws InterruptedException {
        joint.setPosition(posJointIdle.POS_SERVO);
        sleep(200);
        motor.setTargetPosition(posGround2.POS_MOTOR);
        motor.setPower(0.4);
        sleep(700);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(posGround.POS_MOTOR);
        motor.setPower(0.1);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        jointStatus = JointStatus.IDLE;
    }

    public void updateJoint() {
        switch (jointStatus) {
            case IDLE: {
                joint.setPosition(posJointIdle.POS_SERVO);
                jointStatus = JointStatus.GRAB;
                break;
            }
            case GRAB: {
                joint.setPosition(posGround.POS_SERVO);
                jointStatus = JointStatus.IDLE;
                break;
            }
            case AUTO: {
                joint.setPosition(0.45);
            }
            case INIT_AUTO: {
                joint.setPosition(0.39);
            }
        }
    }
}
