package org.firstinspires.ftc.teamcode.teamcode.opencv;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp(group = "Tests")
@Config
public class testOpenCv extends LinearOpMode {
    public CameraDetector camera;
    @Override
    public void runOpMode() throws InterruptedException {
        camera = new CameraDetector(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));

        waitForStart();

        while (opModeIsActive())
        {
            CameraDetector.Result randResult = camera.detect();
            telemetry.addLine("DETECTED: " + randResult);
            telemetry.update();

        }

    }
}
