package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.hardware.Brat;
import org.firstinspires.ftc.teamcode.teamcode.hardware.Cleste;
import org.firstinspires.ftc.teamcode.teamcode.hardware.Flipper;
import org.firstinspires.ftc.teamcode.teamcode.hardware.StickyGamepad;
import org.firstinspires.ftc.teamcode.teamcode.hardware.lift;

import java.util.List;

@TeleOp
public class teleop_kickoff extends LinearOpMode {

    public SampleMecanumDrive drive;
    public StickyGamepad stickyGamepad1;
    public StickyGamepad stickyGamepad2;
    DcMotorEx leftFront, leftRear, rightFront, rightRear;
    Brat brat;
    Cleste cleste;
    public DcMotorEx huuuuuStanga, huuuuuDreapta;

    public void move(Gamepad g)
    {
        double rotRight = 0, rotLeft = 0;
        if(g.right_stick_x > 0) {
            rotRight = g.right_stick_x;
        } else {
            rotLeft = 0 - g.right_stick_x;
        }
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear.setPower(-g.left_stick_y + g.left_stick_x - rotRight + rotLeft);
        rightFront.setPower(-g.left_stick_y - g.left_stick_x - rotRight + rotLeft);
        leftFront.setPower(-g.left_stick_y + g.left_stick_x + rotRight - rotLeft);
        leftRear.setPower(-g.left_stick_y - g.left_stick_x + rotRight - rotLeft);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        huuuuuStanga = hardwareMap.get(DcMotorEx.class, "lansatorStanga");
        huuuuuDreapta = hardwareMap.get(DcMotorEx.class, "lansatorDreapta");

        huuuuuDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        huuuuuStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        huuuuuStanga.setDirection(DcMotorSimple.Direction.REVERSE);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        stickyGamepad1 = new StickyGamepad(gamepad1, this);
        stickyGamepad2 = new StickyGamepad(gamepad2, this);
        drive = new SampleMecanumDrive(hardwareMap);
        brat = new Brat(hardwareMap, "brat", "joint");
        brat.moveToInit();
        brat.jointStatus = Brat.JointStatus.INIT;
        cleste = new Cleste(hardwareMap, "cleste");
        cleste.fullyOpenGripper();
        waitForStart();
        brat.moveToGame();
        while(opModeIsActive() && !isStopRequested()) {

            drive.update();
            move(gamepad1);
//            if(gamepad1.left_trigger != 0)
//            {
//                brat.CurrentState = Brat.state.bratGoingDown;
//            } else if(gamepad1.right_trigger != 0)
//            {
//                brat.CurrentState = Brat.state.bratGoingUp;
//            } else
//
            if(gamepad1.dpad_down)
            {
                brat.CurrentState = Brat.state.jointGoingDown;
            } else if(gamepad1.dpad_up)
            {
                brat.CurrentState = Brat.state.jointGoingUp;
            }

            if(gamepad1.touchpad)
            {
                huuuuuDreapta.setPower(1);
                huuuuuStanga.setPower(1);
            } else
            {
                huuuuuDreapta.setPower(0);
                huuuuuStanga.setPower(0);
            }

            if(gamepad1.a)
            {
                brat.CurrentState = Brat.state.goToGround;
            }

            if(gamepad1.y)
            {
                brat.CurrentState = Brat.state.goToBoard;
            }

            if(gamepad1.right_bumper)
            {
                cleste.update();
                sleep(100);
            }

            if(gamepad1.left_bumper)
            {
                brat.CurrentState = Brat.state.gotoGrab;
            }

            if(gamepad1.b) {
                brat.updateJoint();
                sleep(100);
            }

            brat.update();
            telemetry.addData("pozBrat", brat.motor.getCurrentPosition());
            telemetry.addData("brat ", brat.CurrentState);
            telemetry.addData("cleste ", cleste.status);
            telemetry.addData("joint ", brat.jointStatus);
            telemetry.update();
        }
    }
}
