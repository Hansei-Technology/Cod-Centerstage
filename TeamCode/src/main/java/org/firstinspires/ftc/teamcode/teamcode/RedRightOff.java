package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.ExtensionController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.JointController;
import org.firstinspires.ftc.teamcode.teamcode.controllers.UniversalStates;

@Autonomous
public class RedRightOff extends LinearOpMode {

    public static Pose2d START_POSE = new Pose2d(0, 0, 0);
    public SampleMecanumDrive drive;
    public ClawController clawController;
    public ArmController armController;
    public ExtensionController extensionController;
    public JointController jointController;
    public UniversalStates robot;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new UniversalStates();
        armController = new ArmController(hardwareMap, robot);
        if(isStopRequested()) return;
        drive.setPoseEstimate(START_POSE);

    }

    public TrajectorySequence buildMainTraj(){
        TrajectorySequenceBuilder trajectorySequenceBuilder = new TrajectorySequenceBuilder(START_POSE);
        trajectorySequenceBuilder.lineToConstantHeading(new Vector2d(10, 10))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.state = UniversalStates.State.ARM_UP;
                    clawController.statusRight = ClawController.GripperStatus.CLOSED;
                    clawController.update();
                    armController.update();
                });
        return trajectorySequenceBuilder.build();
    }

}
