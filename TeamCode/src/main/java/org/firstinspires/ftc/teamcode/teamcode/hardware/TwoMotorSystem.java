package org.firstinspires.ftc.teamcode.teamcode.hardware;

import static java.lang.Double.max;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Aceasta clasa reprezinta un sistem de 2 motoare unde al doilea este in direction REVERSE
public abstract class TwoMotorSystem
{
    public DcMotorEx motorLeft, motorRight;

    public TwoMotorSystem(HardwareMap map, String nume1, String nume2)
    {
        motorLeft = map.get(DcMotorEx.class, nume1);
        motorRight = map.get(DcMotorEx.class, nume2);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour)
    {
        motorLeft.setZeroPowerBehavior(behaviour);
        motorRight.setZeroPowerBehavior(behaviour);
    }

    public void setMode(DcMotor.RunMode mode)
    {
        motorLeft.setMode(mode);
        motorRight.setMode(mode);
    }

    public void setTargetPosition(int position)
    {
        motorLeft.setTargetPosition(position);
        motorRight.setTargetPosition(position);

        /*
        setMode(RUN_WITHOUT_ENCODER);
        if(position > getCurrentPosition())
        {
            functie1(position); //merge in sus
        } else
        {
            functie2(position); //merge in jos
        }
        setPower(0);
        */
    }

    double f(double x) //functie de gr 2, folosim pt grafic
    {
        return -4 * x * x + 4 * x;
    }

    void functie1(double target) //in sus
    {
        double distTotal = target - getCurrentPosition();
        double distAcc = (target - getCurrentPosition()) / distTotal; //intre 0 si 1
        while(distAcc > 0)
        {
            distAcc = (target - getCurrentPosition()) / distTotal;
            setPower(max(f(distAcc), 0.3)); //Pmin = 0.3
        }
    }

    void functie2(double target) //in jos
    {
        double distTotal = getCurrentPosition() - target;
        double distAcc = (getCurrentPosition() - target) / distTotal; //intre 0 si 1
        while(distAcc > 0)
        {
            distAcc = (getCurrentPosition() - target) / distTotal;
            setPower(-1 * max(f(distAcc), 0.3)); //Pmin = 0.3
        }
    }

    public void setPower(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void setVelocity(double angularRate) {
        motorLeft.setVelocity(angularRate);
        motorRight.setVelocity(angularRate);
    }

    public double getCurrentPosition()
    {
        return motorLeft.getCurrentPosition();
    }
    
    public double getTargetPosition() { return motorLeft.getTargetPosition(); }
    
    public boolean isBusy()
    {
        return motorLeft.isBusy();
    }

}
