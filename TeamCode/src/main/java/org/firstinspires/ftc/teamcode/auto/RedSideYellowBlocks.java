package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;

@Autonomous(name="Red - Yellow Blocks", group="Hippos")
public class RedSideYellowBlocks extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-36, -60, Math.PI/2);
        Pose2d rightBlockApproachPose = new Pose2d(-40, -12, Math.PI/2);
        Pose2d rightBlockMovePose = new Pose2d(-60, -60, -Math.PI/2);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

//        Action rightBlockApproach = drive.actionBuilder(beginPose)
//                .splineTo(rightBlockApproachPose.position, 0)
//                .build();
//
//        Action rightBlockMove = drive.actionBuilder(rightBlockApproachPose)
//                .splineTo(rightBlockApproachPose.position, 0)
//                .build();

        TrajectoryActionBuilder rightBlock = drive.actionBuilder(beginPose)
                .splineTo(rightBlockApproachPose.position, Math.PI/2)
                .turnTo(Math.toRadians(0))
                .splineTo(rightBlockMovePose.position, -Math.PI/2);

        TrajectoryActionBuilder rightTurn90Trajectory = drive.actionBuilder(rightBlockMovePose)
                .turn(Math.toRadians(90));


        Action trajectory = rightBlock.build();
        Action rightTurn90 = rightTurn90Trajectory.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        trajectory
                )
        );

    }
}