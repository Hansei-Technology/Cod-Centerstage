package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ExtensionController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.JointController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.UniversalStates;
import org.firstinspires.ftc.teamcode.teamcode.opencv.CameraDetector;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
@Config
public class RedRightOff extends LinearOpMode {

    public static Pose2d START_POSE = new Pose2d(0, 0, 0);
    public SampleMecanumDrive drive;
    public CameraDetector camera;
    public ClawController clawController;
    public ArmController armController;
    public ExtensionController extensionController;
    public JointController jointController;
    public UniversalStates robot;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new CameraDetector(OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")));
        robot = new UniversalStates();
        armController = new ArmController(hardwareMap, robot);
        clawController = new ClawController(hardwareMap);
        TrajectorySequence mainTrajectory = null;
        while(opModeInInit()){
            CameraDetector.Result result=camera.detect();
            telemetry.addLine("Location" + result);
            telemetry.update();
            mainTrajectory = buildMainTrajectory(result);
            robot.state = UniversalStates.State.MOVING;
            armController.update();
        }
        camera.stop();
        if(isStopRequested()) return;
        drive.setPoseEstimate(START_POSE);
        drive.followTrajectorySequence(mainTrajectory);

    }

    public TrajectorySequence buildMainTrajectory(CameraDetector.Result result){
        TrajectorySequenceBuilder mainTrajectoryBuilder = drive.trajectorySequenceBuilder(START_POSE);
        switch (result){
            case CENTER: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        })

                        .lineToLinearHeading(new Pose2d(23, -6, Math.toRadians(0)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(0)));
                return  mainTrajectoryBuilder.build();
            }
            case RIGHT: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        })
                        .lineToLinearHeading(new Pose2d(15, -15, Math.toRadians(0)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(0)));
                return mainTrajectoryBuilder.build();
            }
            case LEFT: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        })
                        .lineToLinearHeading(new Pose2d(18, 15, Math.toRadians(320)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(0)));
                return mainTrajectoryBuilder.build();
            }
            case NONE: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                        })
                        .lineToLinearHeading(new Pose2d(23, -6, Math.toRadians(0)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(10, -62, Math.toRadians(0)));
                return mainTrajectoryBuilder.build();
            }
        }
        return  mainTrajectoryBuilder.build();
    }
    }


