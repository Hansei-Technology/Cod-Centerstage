package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ExtensionController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.JointController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.UniversalStates;
import org.firstinspires.ftc.teamcode.teamcode.opencv.CameraDetector;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
@Config
public class RedHigh extends LinearOpMode {

    public static Pose2d EOOK = new Pose2d(20, -5, Math.toRadians(0));
    public static Pose2d LEFT = new Pose2d(22.5, 2.3, Math.toRadians(90));
    public static Pose2d RIGHT = new Pose2d(17, -5.6, Math.toRadians(354.3));
    public static Pose2d MID = new Pose2d(24.8, 1, Math.toRadians(0));
    public static Pose2d BACKBOARD = new Pose2d(26, -29.5, Math.toRadians(90));
    public static Pose2d BACKBOARD_LEFT = new Pose2d(30, -29.5, Math.toRadians(90));
    public static Pose2d BACKBOARD_RIGHT = new Pose2d(20, -29.5, Math.toRadians(90));
    public static Pose2d PARK = new Pose2d(50, -29.5, Math.toRadians(90));
    //public static Pose2d TRUSS = new Pose2d(20.34, 7.82, Math.toRadians(52.8));
    public static Pose2d STACK = new Pose2d(11.27, 54.9, Math.toRadians(114.3));
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
        jointController = new JointController(hardwareMap, robot);
        extensionController = new ExtensionController(hardwareMap, robot);
        TrajectorySequence mainTrajectory = null;
        TrajectorySequence secondTrajectory = null;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        clawController.statusRight = ClawController.GripperStatus.CLOSED;
        clawController.statusLeft = ClawController.GripperStatus.CLOSED;
        clawController.update();
        sleep(1000);
        robot.state = UniversalStates.State.AUTO;
        jointController.goToAuto();
        armController.goToPos(1500);
        CameraDetector.Result result = CameraDetector.Result.NONE;
        while (opModeInInit()) {
            result = camera.detect();
            telemetry.addLine("Location" + result);
            telemetry.update();
        }
        camera.stop();
        mainTrajectory = buildMainTrajectory(result);
        if (isStopRequested()) return;
        drive.setPoseEstimate(START_POSE);
        drive.followTrajectorySequence(mainTrajectory);
    }

    public TrajectorySequence buildSecondTrajectory() {
        TrajectorySequenceBuilder secondTrajectoryBuilder = drive.trajectorySequenceBuilder(buildMainTrajectory(CameraDetector.Result.CENTER).end());
        secondTrajectoryBuilder
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    jointController.goToPos0();
                    extensionController.goToPos(0);
                    armController.goToPos(400);
                })
                .lineToLinearHeading(new Pose2d(65, 30, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(65, 150, Math.toRadians(90)))
                .lineTo(
                        new Vector2d(60, 180),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    extensionController.goToPos(0);
                    armController.goToPos(400);
                    jointController.goToPos0();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    clawController.statusRight = ClawController.GripperStatus.CLOSED;
                    clawController.update();
                })
                .lineToLinearHeading(new Pose2d(65, 30, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    extensionController.goToPos(400);
                    armController.goToPos(4550 - 1500);
                    jointController.goToPos0();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    clawController.statusRight = ClawController.GripperStatus.OPEN;
                    clawController.update();
                });
        return secondTrajectoryBuilder.build();
    }

    public TrajectorySequence buildMainTrajectory(CameraDetector.Result result) {
        TrajectorySequenceBuilder mainTrajectoryBuilder = drive.trajectorySequenceBuilder(START_POSE);
        switch (result) {

            case LEFT: {
                mainTrajectoryBuilder
                        .waitSeconds(0.3)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            armController.goToPos(400);
                            jointController.goToPos0();
                        })
                        .lineToLinearHeading(EOOK)
                        .lineToLinearHeading(LEFT)
                        .addSpatialMarker(LEFT.vec(), () -> {
                            clawController.statusRight = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .waitSeconds(0.5)
                        .lineToLinearHeading(BACKBOARD)
                        .addSpatialMarker(BACKBOARD.vec(), () -> {
                            extensionController.goToPos(extensionController.POZ0);
                            armController.goToPos(3200);
                            jointController.goToBackboard();
                        })
                        .lineToLinearHeading(BACKBOARD_LEFT)
                        .waitSeconds(0.5)
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            clawController.statusLeft = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                            armController.goToPos(400);
                            extensionController.goToPos(0);
                            jointController.goToPos0();
                        })
                        .waitSeconds(0.5)
                        .lineToLinearHeading(PARK)
                        .addSpatialMarker(PARK.vec(), () -> {
                            jointController.endAuto();
                        });

                return mainTrajectoryBuilder.build();
            }

            case CENTER: {

                mainTrajectoryBuilder
                        .waitSeconds(0.3)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            armController.goToPos(400);
                            jointController.goToPos0();
                        })
                        .lineToLinearHeading(MID)
                        .addSpatialMarker(MID.vec(), () -> {
                            clawController.statusRight = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        //.waitSeconds(0.5)
                        .lineToLinearHeading(BACKBOARD)
                        .addSpatialMarker(BACKBOARD.vec(), () -> {
                            extensionController.goToPos(extensionController.POZ0);
                            armController.goToPos(3200);
                            jointController.goToBackboard();
                        })
                        .waitSeconds(0.5)
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            clawController.statusLeft = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                            armController.goToPos(400);
                            extensionController.goToPos(0);
                            jointController.goToPos0();
                        })
                        .waitSeconds(0.5)
                        .lineToLinearHeading(PARK)
                        .addSpatialMarker(PARK.vec(), () -> {
                            jointController.endAuto();
                        });

                return mainTrajectoryBuilder.build();
            }

            case RIGHT: {
                mainTrajectoryBuilder
                        .waitSeconds(0.3)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            armController.goToPos(400);
                            jointController.goToPos0();
                        })
                        .lineToLinearHeading(RIGHT)
                        .addSpatialMarker(RIGHT.vec(), () -> {
                            clawController.statusRight = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        //.waitSeconds(0.5)
                        .lineToLinearHeading(BACKBOARD)
                        .addSpatialMarker(BACKBOARD.vec(), () -> {
                            extensionController.goToPos(extensionController.POZ0);
                            armController.goToPos(3200);
                            jointController.goToBackboard();
                        })
                        .lineToLinearHeading(BACKBOARD_RIGHT)
                        .waitSeconds(0.5)
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            clawController.statusLeft = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                            armController.goToPos(400);
                            extensionController.goToPos(0);
                            jointController.goToPos0();
                        })
                        .waitSeconds(0.5)
                        .lineToLinearHeading(PARK)
                        .addSpatialMarker(PARK.vec(), () -> {
                            jointController.endAuto();
                        });
                return mainTrajectoryBuilder.build();
            }

            case NONE: {
                mainTrajectoryBuilder
                        .waitSeconds(0.3)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            armController.goToPos(400);
                            jointController.goToPos0();
                        })
                        .lineToLinearHeading(MID)

                        .UNSTABLE_addTemporalMarkerOffset(0.14, () -> {
                            clawController.statusRight = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            extensionController.goToPos(500);
                            armController.goToPos(3050);
                            jointController.goToPos0();
                        })
                        .lineToLinearHeading(BACKBOARD)
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            clawController.statusLeft = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                            armController.goToPos(400);
                            extensionController.goToPos(0);
                        })
                        .waitSeconds(1.5)
                        .lineToLinearHeading(PARK);
                return mainTrajectoryBuilder.build();
            }
        }
        return  mainTrajectoryBuilder.build();
    }
}


