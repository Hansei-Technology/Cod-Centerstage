package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.opencv.core.Mat;

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

    public double speed;

    public void move(Gamepad g)
    {
        double rotRight = 0, rotLeft = 0;
        if(g.right_stick_x > 0) {
            rotRight = g.right_stick_x;
        } else {
            rotLeft = 0 - g.right_stick_x;
        }

        if(Math.abs(g.left_stick_x) < 0.14)
            g.left_stick_x = 0;
        if(Math.abs(g.left_stick_y) < 0.14)
            g.left_stick_y = 0;

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear.setPower((-g.left_stick_y + g.left_stick_x - rotRight + rotLeft) * speed);
        rightFront.setPower((-g.left_stick_y - g.left_stick_x - rotRight + rotLeft) * speed);
        leftFront.setPower((-g.left_stick_y + g.left_stick_x + rotRight - rotLeft) * speed);
        leftRear.setPower((-g.left_stick_y - g.left_stick_x + rotRight - rotLeft) * speed);
    }

    public void FieldCentricDrive(Gamepad g, double power, double angle) {

        double y = -g.left_stick_y;
        double x = g.left_stick_x;
        double rx = g.right_stick_x;
        double rotX = x * Math.cos(-angle) - y * Math.sin(-angle);
        double rotY = x * Math.sin(-angle) - y * Math.cos(-angle);

        double denom = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);

        double lf, lr, rf, rr, maxS;
        rf = (rotY - rotX - rx) / denom;
        lf = (rotY + rotX + rx) / denom;
        rr = (rotY + rotX - rx) / denom;
        lr = (rotY - rotX + rx) / denom;

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setPower(rf * power);
        leftFront.setPower(lf * power);
        rightRear.setPower(rr * power);
        leftRear.setPower(lr * power);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        speed = 0.5;

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
            //FieldCentricDrive(gamepad1, 0.5, drive.getExternalHeading());


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
            } else
            {
                brat.CurrentState = Brat.state.noUpdate;
            }

            if(gamepad1.touchpad)
            {
                huuuuuDreapta.setPower(1);
//                huuuuuStanga.setPower(1);
            } else
            {
                huuuuuDreapta.setPower(0);
//                huuuuuStanga.setPower(0);
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
            telemetry.addData("heading ", drive.getExternalHeading());
            telemetry.addData("cleste ", cleste.status);
            telemetry.addData("joint ", brat.jointStatus);
            telemetry.update();
        }
    }
}
