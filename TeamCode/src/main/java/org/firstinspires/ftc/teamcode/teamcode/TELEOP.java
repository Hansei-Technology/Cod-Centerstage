package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.JointController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ExtensionController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ChassisController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.PlaneLauncher;
import org.firstinspires.ftc.teamcode.teamcode.controllers.UniversalStates;
import org.firstinspires.ftc.teamcode.teamcode.util.StickyGamepad;

import java.util.List;

@TeleOp
public class TELEOP extends LinearOpMode {

    ClawController claw;
    JointController joint;
    ArmController arm;
    ExtensionController extension;
    ChassisController chassis;
    UniversalStates robot;
    PlaneLauncher avion;

    StickyGamepad stickyG1;
    StickyGamepad stickyG2;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new UniversalStates();
        claw = new ClawController(hardwareMap);
        joint = new JointController(hardwareMap, robot);
        extension = new ExtensionController(hardwareMap, robot);
        chassis = new ChassisController(hardwareMap);
        arm = new ArmController(hardwareMap, robot);
        avion = new PlaneLauncher(hardwareMap);

        stickyG1 = new StickyGamepad(gamepad1, this);
        stickyG2 = new StickyGamepad(gamepad2, this);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        while(opModeIsActive()) {
            chassis.Move(gamepad1);
            stickyG1.update();
            stickyG2.update();

            if(stickyG2.left_bumper) //leftClaw
            {
                if(claw.statusLeft == ClawController.GripperStatus.CLOSED) {
                    claw.statusLeft = ClawController.GripperStatus.OPEN;
                    if(robot.state==UniversalStates.State.ARM_UP) {
                        robot.state= UniversalStates.State.MOVING;
                    }
                }else
                    claw.statusLeft = ClawController.GripperStatus.CLOSED;
            }
            if(stickyG2.right_bumper) //rightClaw
            {
                if(claw.statusRight == ClawController.GripperStatus.CLOSED)
                    claw.statusRight = ClawController.GripperStatus.OPEN;
                else
                    claw.statusRight = ClawController.GripperStatus.CLOSED;
            }

            //vrea si bogdy sa deschida clestele
            if(stickyG1.left_bumper) //leftClaw
            {
                claw.statusLeft = ClawController.GripperStatus.OPEN;
            }
            if(stickyG1.right_bumper) //rightClaw
            {
                claw.statusRight = ClawController.GripperStatus.OPEN;
            }

            //armControl
            if(gamepad1.a) {
                if(robot.previousState != robot.state) {
                    extension.targetPos = 0;
                }
                robot.state = UniversalStates.State.ARM_DOWN;
            }
            if(gamepad1.y) {
                if(robot.previousState != robot.state) {
                    robot.currentRow = 1;
                }
                robot.state = UniversalStates.State.ARM_UP;
            }
            if(gamepad1.b) {
                if(robot.previousState != robot.state) {
                    extension.targetPos = 0;
                }
                robot.state = UniversalStates.State.MOVING;
            }

            //extensonControl when ARM_DOWN
            if(gamepad2.left_trigger != 0) {
                extension.targetPos -= 20;
            }
            if(gamepad2.right_trigger != 0) {
                extension.targetPos += 20;
            }

            //backdrop row selector
            if(gamepad2.a) robot.currentRow = 4;
            if(gamepad2.b) robot.currentRow = 7;
            if(gamepad2.y) robot.currentRow = 10;
            if(stickyG2.dpad_up) robot.currentRow++;
            if(stickyG2.dpad_down) robot.currentRow--;
            robot.currentRow = Math.min(11, Math.max(robot.currentRow, 1));

            if(gamepad1.touchpad) avion.Lauch();

            //WARNING: UGLY CODE
            if(stickyG1.dpad_right) {
                if(robot.state == UniversalStates.State.READY_TO_HANG) {
                    extension.targetPos = extension.HANG - 400; //TODO: SET THIS
                } else {
                    arm.motor.setTargetPosition(arm.HANG);
                    arm.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.motor.setPower(1);
                    extension.targetPos = extension.HANG;
                    robot.state = UniversalStates.State.READY_TO_HANG;
                }
            }

            extension.update();
            if(robot.state != UniversalStates.State.READY_TO_HANG)
            {
                claw.update();
                joint.update();
                arm.update();
            }

            robot.previousState = robot.state;

            telemetry.addData("robot : ", robot.state);
            telemetry.addData("clawLeft", claw.statusLeft);
            telemetry.addData("clawRight", claw.statusRight);
            telemetry.addData("row", robot.currentRow);
            telemetry.addData("extensionPos", robot.extensionPos);
            telemetry.addData("armPos", arm.currentPosition);
            telemetry.update();
        }
    }
}
