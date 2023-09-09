package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.hardware.StickyGamepad;

import java.util.List;

@TeleOp
public class teleop_kickoff extends LinearOpMode {

    public SampleMecanumDrive drive;
    public StickyGamepad stickyGamepad1;
    public StickyGamepad stickyGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        stickyGamepad1 = new StickyGamepad(gamepad1, this);
        stickyGamepad2 = new StickyGamepad(gamepad2, this);
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive() && isStopRequested()) {
            drive.update();
        }
    }
}
