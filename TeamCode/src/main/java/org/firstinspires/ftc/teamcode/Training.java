package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamcode.hardware.StickyGamepad;

@TeleOp
public class Training extends LinearOpMode {

    DcMotorEx leftFront, leftRear, rightFront, rightRear, brat;
    Servo joint, cleste;

    StickyGamepad stickyGamepad1;

    //declaram variabile
    @Override
    public void runOpMode() throws InterruptedException {

        stickyGamepad1 = new StickyGamepad(gamepad1, this);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        brat = hardwareMap.get(DcMotorEx.class, "brat");

        joint = hardwareMap.get(Servo.class, "joint");
        cleste = hardwareMap.get(Servo.class, "cleste");

        joint.setPosition(0.45);

        waitForStart();

        while(opModeIsActive())
        {
            leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 2.5);
            leftRear.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / 2.5);
            rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / 2.5);
            rightRear.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / 2.5);
            //cod miscare


            if(gamepad1.y) //bratul se duce sus
            {
                brat.setTargetPosition(120);
                brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                brat.setPower(0.7);
            }
            if(gamepad1.a) //bratul se duce jos
            {
                brat.setTargetPosition(15);
                brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                brat.setPower(0.7);
            }

            if(gamepad1.dpad_up) //joint se duce in sus
            {
                joint.setPosition(joint.getPosition() - 0.01);
                sleep(100);
                telemetry.addData("sus", " ");
                telemetry.update();
            } else if(gamepad1.dpad_down) //joint se duce in jos
            {
                joint.setPosition(joint.getPosition() + 0.01);
                sleep(100);
                telemetry.addData("sus", " ");
                telemetry.update();
            }

            if(gamepad1.left_bumper) //cleste deschis
            {
                cleste.setPosition(0.5);
            } else if(gamepad1.right_bumper) //cleste inchis
            {
                cleste.setPosition(0.8);
            }
        }
    }
}
