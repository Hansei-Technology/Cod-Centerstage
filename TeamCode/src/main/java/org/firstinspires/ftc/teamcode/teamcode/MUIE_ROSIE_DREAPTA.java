package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.fasterxml.jackson.databind.JsonSerializer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.teamcode.hardware.Brat;
import org.firstinspires.ftc.teamcode.teamcode.hardware.Cleste;
import org.firstinspires.ftc.teamcode.teamcode.opencv.CameraDetector;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
@Config
public class MUIE_ROSIE_DREAPTA extends LinearOpMode {
    public static Pose2d START_POSE = new Pose2d(0, 0, 0);
    public SampleMecanumDrive drive;
    public CameraDetector camera;
    public Brat brat;
    public Cleste cleste;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new CameraDetector(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));
        brat = new Brat(hardwareMap, "brat", "joint");
        cleste = new Cleste(hardwareMap, "cleste");
        TrajectorySequence mainTrajectory = null;
        cleste.closeGripper();
        brat.jointStatus = Brat.JointStatus.AUTO;
        brat.updateJoint();
        while(opModeInInit()) {
            CameraDetector.Result result = camera.detect();
            telemetry.addLine("Location " + result);
            telemetry.update();
            mainTrajectory = buildMainTrajectory(result);
        }
        camera.stop();
        if (isStopRequested()) return;
        drive.setPoseEstimate(START_POSE);
        drive.followTrajectorySequence(mainTrajectory);
    }

    public TrajectorySequence buildMainTrajectory(CameraDetector.Result result) {
        TrajectorySequenceBuilder mainTrajectoryBuilder = drive.trajectorySequenceBuilder(START_POSE);
        switch (result){
            case CENTER: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(1.8, () -> cleste.openGripper())
                        .lineToLinearHeading(new Pose2d(23, -6, Math.toRadians(0)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(0)));
                return  mainTrajectoryBuilder.build();
            }
            case RIGHT: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(1.8, () -> cleste.openGripper())
                        .lineToLinearHeading(new Pose2d(15, -10, Math.toRadians(0)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(10, -58, Math.toRadians(0)));
                return mainTrajectoryBuilder.build();
            }
            case LEFT: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(1.8, () -> cleste.openGripper())
                        .lineToLinearHeading(new Pose2d(18, 13, Math.toRadians(315)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(10, -53, Math.toRadians(0)));
                return mainTrajectoryBuilder.build();
            }

        }
        mainTrajectoryBuilder
                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> cleste.openGripper())
                .lineToLinearHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(10, 58, Math.toRadians(0)));
        return  mainTrajectoryBuilder.build();
    }
}