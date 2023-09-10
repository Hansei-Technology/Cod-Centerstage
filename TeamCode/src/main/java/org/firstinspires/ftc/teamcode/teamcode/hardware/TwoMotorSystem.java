package org.firstinspires.ftc.teamcode.hardware.base;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Double.max;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Aceasta clasa reprezinta un sistem de 2 motoare unde al doilea este in direction REVERSE
public abstract class TwoMotorSystem
{
    public DcMotorEx motor1, motor2;

    public TwoMotorSystem(HardwareMap map, String nume1, String nume2)
    {
        motor1 = map.get(DcMotorEx.class, nume1);
        motor2 = map.get(DcMotorEx.class, nume2);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour)
    {
        motor1.setZeroPowerBehavior(behaviour);
        motor2.setZeroPowerBehavior(behaviour);
    }

    public void setMode(DcMotor.RunMode mode)
    {
        motor1.setMode(mode);
        motor2.setMode(mode);
    }

    public void setTargetPosition(int position)
    {
        motor1.setTargetPosition(position);
        motor2.setTargetPosition(position);

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
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void setVelocity(double angularRate) {
        motor1.setVelocity(angularRate);
        motor2.setVelocity(angularRate);
    }

    public double getCurrentPosition()
    {
        return motor1.getCurrentPosition();
    }
    
    public double getTargetPosition() { return motor1.getTargetPosition(); }
    
    public boolean isBusy()
    {
        return motor1.isBusy();
    }

}
