package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(group = "Tests")
@Config
public class TestConfigServo extends LinearOpMode
{
    public Servo servo;
    AnalogInput encoder;
    public static String nServo = "sleeveS";
    public static String nEncoder = "AnalogInput1";

    public static double power = 0;

    @Override
    public void runOpMode(){

        servo = this.hardwareMap.get(Servo.class, nServo);
        encoder = this.hardwareMap.get(AnalogInput.class, nEncoder);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()){

            servo.setPosition(power);
            double pos = encoder.getVoltage() / 3.3 * 360;
            telemetry.addData("pos", pos);
            telemetry.update();
        }

    }

}
