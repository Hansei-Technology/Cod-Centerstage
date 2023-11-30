package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
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
import org.opencv.core.Mat;
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
        jointController = new JointController(hardwareMap,robot);
        TrajectorySequence mainTrajectory = null;

        clawController.statusRight = ClawController.GripperStatus.CLOSED;
        clawController.statusLeft = ClawController.GripperStatus.CLOSED;
        clawController.update();
        sleep(1000);
        robot.state = UniversalStates.State.AUTO;
        jointController.update();
        armController.update();

        while(opModeInInit()){
            CameraDetector.Result result=camera.detect();
            telemetry.addLine("Location" + result);
            telemetry.update();
            mainTrajectory = buildMainTrajectory(result);
        }
        camera.stop();
        if(isStopRequested()) return;
        drive.setPoseEstimate(START_POSE);
        drive.followTrajectorySequence(mainTrajectory);

    }

    public TrajectorySequence buildMainTrajectory(CameraDetector.Result result){
        TrajectorySequenceBuilder mainTrajectoryBuilder = drive.trajectorySequenceBuilder(START_POSE);
        switch (result){

            case LEFT: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        })

                        .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0.6,() ->{
                            clawController.statusRight = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(20,-22,Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
                            robot.state = UniversalStates.State.ARM_UP;
                            robot.currentRow = 4;
                            extensionController.update();
                            armController.update();
                        })
                        .waitSeconds(0)
                        .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                            clawController.statusLeft = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.4,() ->{
//                            robot.state = UniversalStates.State.MOVING;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .lineToLinearHeading(new Pose2d(65,30,Math.toRadians(90)))
//                        .lineToLinearHeading(new Pose2d(65,150, Math.toRadians(90)))
//                        .lineTo(
//                                new Vector2d(60, 180),
//                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        )
//                        .UNSTABLE_addTemporalMarkerOffset(4,() ->{
//                            robot.state = UniversalStates.State.AUTO;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.4,() ->{
//                            clawController.statusRight = ClawController.GripperStatus.CLOSED;
//                            clawController.update();
//                        })
//                        .lineToLinearHeading(new Pose2d(65,30,Math.toRadians(90)))
//                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
//                            robot.state = UniversalStates.State.ARM_UP;
//                            robot.currentRow = 4;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
//                            clawController.statusRight = ClawController.GripperStatus.OPEN;
//                            clawController.update();
//                        })
                        .lineToLinearHeading(new Pose2d(25,-13,Math.toRadians(90)));
                return  mainTrajectoryBuilder.build();
            }

            case CENTER: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        })

                        .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0.6,() ->{
                            clawController.statusRight = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(20,-20,Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
                            robot.state = UniversalStates.State.ARM_UP;
                            robot.currentRow = 4;
                            extensionController.update();
                            armController.update();
                        })
                        .waitSeconds(0)
                        .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                            clawController.statusLeft = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        //                       .UNSTABLE_addTemporalMarkerOffset(0.4,() ->{
//                            robot.state = UniversalStates.State.MOVING;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .lineToLinearHeading(new Pose2d(65,30,Math.toRadians(90)))
//                        .lineToLinearHeading(new Pose2d(65,150, Math.toRadians(90)))
//                        .lineTo(
//                                new Vector2d(60, 180),
//                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        )
//                        .UNSTABLE_addTemporalMarkerOffset(4,() ->{
//                            robot.state = UniversalStates.State.AUTO;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.4,() ->{
//                            clawController.statusRight = ClawController.GripperStatus.CLOSED;
//                            clawController.update();
//                        })
//                        .lineToLinearHeading(new Pose2d(65,30,Math.toRadians(90)))
//                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
//                            robot.state = UniversalStates.State.ARM_UP;
//                            robot.currentRow = 4;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
//                            clawController.statusRight = ClawController.GripperStatus.OPEN;
//                            clawController.update();
//                        })
                        .lineToLinearHeading(new Pose2d(25,-13,Math.toRadians(90)));
                ;
                return  mainTrajectoryBuilder.build();
            }

            case RIGHT: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        })

                        .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0.6,() ->{
                            clawController.statusRight = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(20,-21,Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
                            robot.state = UniversalStates.State.ARM_UP;
                            robot.currentRow = 4;
                            extensionController.update();
                            armController.update();
                        })
                        .waitSeconds(0)
                        .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                            clawController.statusLeft = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        //                       .UNSTABLE_addTemporalMarkerOffset(0.4,() ->{
//                            robot.state = UniversalStates.State.MOVING;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .lineToLinearHeading(new Pose2d(65,30,Math.toRadians(90)))
//                        .lineToLinearHeading(new Pose2d(65,150, Math.toRadians(90)))
//                        .lineTo(
//                                new Vector2d(60, 180),
//                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        )
//                        .UNSTABLE_addTemporalMarkerOffset(4,() ->{
//                            robot.state = UniversalStates.State.AUTO;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.4,() ->{
//                            clawController.statusRight = ClawController.GripperStatus.CLOSED;
//                            clawController.update();
//                        })
//                        .lineToLinearHeading(new Pose2d(65,30,Math.toRadians(90)))
//                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
//                            robot.state = UniversalStates.State.ARM_UP;
//                            robot.currentRow = 4;
//                            extensionController.update();
//                            armController.update();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
//                            clawController.statusRight = ClawController.GripperStatus.OPEN;
//                            clawController.update();
//                        })
                        .lineToLinearHeading(new Pose2d(25,-13,Math.toRadians(90)));;
                return  mainTrajectoryBuilder.build();
            }

            case NONE: {
                mainTrajectoryBuilder
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                        })

                        .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0.6,() ->{
                            clawController.statusRight = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(20,-25,Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(0.9,() ->{
                            robot.state = UniversalStates.State.ARM_UP;
                            robot.currentRow = 4;
                            extensionController.update();
                            armController.update();
                        })
                        .waitSeconds(0)
                        .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                            clawController.statusLeft = ClawController.GripperStatus.OPEN;
                            clawController.update();
                        })
                        .lineToLinearHeading(new Pose2d(25,-13,Math.toRadians(90)));

                return  mainTrajectoryBuilder.build();
            }
        }
        return  mainTrajectoryBuilder.build();
    }
    }


