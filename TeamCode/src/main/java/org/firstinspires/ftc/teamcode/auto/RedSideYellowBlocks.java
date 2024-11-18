package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;

@Autonomous(name="Red - Yellow Blocks", group="Hippos")
public class RedSideYellowBlocks extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-48, -48, Math.toRadians(180));
        Pose2d initialBlockDepositPose = new Pose2d(-51.5, -43, Math.toRadians(180));
        Pose2d collectRightBlockPose = new Pose2d(-46, -40, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

//        Action rightBlockApproach = drive.actionBuilder(beginPose)
//                .splineTo(rightBlockApproachPose.position, 0)
//                .build();
//
//        Action rightBlockMove = drive.actionBuilder(rightBlockApproachPose)
//                .splineTo(rightBlockApproachPose.position, 0)
//                .build();

        TrajectoryActionBuilder rightBlock = drive.actionBuilder(beginPose)
                        .splineTo(initialBlockDepositPose.position, Math.toRadians(180))
                        .turn(90)
                        .splineTo(collectRightBlockPose.position, Math.toRadians(90));
        Action trajectory = rightBlock.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        trajectory
                )
        );

    }
}
