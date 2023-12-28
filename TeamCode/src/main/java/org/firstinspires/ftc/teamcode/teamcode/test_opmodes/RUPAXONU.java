package org.firstinspires.ftc.teamcode.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RUPAXONU extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        CRServo servo = hardwareMap.get(CRServo.class, "s1");
        waitForStart();
        while (opModeIsActive())
        {
            servo.setPower(1);
            servo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
}
