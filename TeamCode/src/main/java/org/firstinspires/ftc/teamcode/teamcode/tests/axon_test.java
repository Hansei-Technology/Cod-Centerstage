package org.firstinspires.ftc.teamcode.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class axon_test extends LinearOpMode {
    AnalogInput analog = hardwareMap.get(AnalogInput.class, "Input0");
    Servo servo = hardwareMap.get(Servo.class, "s1");
    boolean sus = false;
    @Override

    public void runOpMode() throws InterruptedException {

        waitForStart();
        servo.setPosition(0);
        while(opModeIsActive())
        {
            if(sus)
            {
                servo.setPosition(servo.getPosition() + 0.05);
                if(servo.getPosition() >= 1)
                    sus = false;
            }
            else
            {
                servo.setPosition(servo.getPosition() - 0.05);
                if(servo.getPosition() <= 0)
                    sus = true;
            }
            wait(100);
            telemetry.addData("poz", analog.getVoltage() / 3.3 * 360);
            telemetry.update();
        }


    }
}
