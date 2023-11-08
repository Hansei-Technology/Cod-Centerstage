package org.firstinspires.ftc.teamcode.SpottyKids;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamcode.hardware.StickyGamepad;
import org.firstinspires.ftc.teamcode.teamcode.hardware.intake;

import java.util.Optional;

@TeleOp
public class Training extends LinearOpMode {
    ChassisController sasiu;
    ArmController brat;
    JointController joint;
    ClawController cleste;

    //declaram variabile
    @Override
    public void runOpMode() throws InterruptedException {

        sasiu = new ChassisController(hardwareMap);
        brat = new ArmController(hardwareMap);
        joint = new JointController(hardwareMap);
        cleste = new ClawController(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            sasiu.move(gamepad1);

            if(gamepad1.y) //bratul se duce sus
            {
                brat.currentState = ArmController.ArmState.UP;
            }
            if(gamepad1.a) //bratul se duce jos
            {
                brat.currentState = ArmController.ArmState.DOWN;
            }

            if(gamepad1.dpad_up) //joint se duce in sus
            {
                joint.currentState = JointController.JointState.GOING_UP;
                sleep(100);
                telemetry.addData("sus", " ");
                telemetry.update();
            } else if(gamepad1.dpad_down) //joint se duce in jos
            {
                joint.currentState = JointController.JointState.GOING_DOWN;
                sleep(100);
                telemetry.addData("sus", " ");
                telemetry.update();
            }

            if(gamepad1.left_bumper) //cleste deschis
            {
                cleste.currentState = ClawController.ClawState.OPEN;
            } else if(gamepad1.right_bumper) //cleste inchis
            {
                cleste.currentState = ClawController.ClawState.CLOSED;
            }

            cleste.update();
            brat.update();
            joint.update();
        }
    }
}
