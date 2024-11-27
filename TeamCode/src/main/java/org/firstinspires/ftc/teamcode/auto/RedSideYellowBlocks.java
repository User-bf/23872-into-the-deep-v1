package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;

@Autonomous(name="Red - Yellow Blocks", group="Hippos")
public class RedSideYellowBlocks extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(-39, -64, Math.toRadians(0));
        Pose2d depositPose = new Pose2d(-60, -60, Math.toRadians(45));
        Pose2d rightBlockPose = new Pose2d(-55, -55, Math.toRadians(80));
        Pose2d centerBlockPose = new Pose2d(-55, -55, Math.toRadians(100));
        Pose2d leftBlockPose = new Pose2d(-55, -55, Math.toRadians(120));
        Pose2d parkPose = new Pose2d(-24, 0, Math.toRadians(90));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, beginPose);
        PinpointDrive drive = robot.drive;

        TrajectoryActionBuilder depositPreloadTrajectory = drive.actionBuilder(beginPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, Math.toRadians(210));

        TrajectoryActionBuilder rightBlockTrajectory = drive.actionBuilder(depositPose)
                .splineToLinearHeading(rightBlockPose, Math.toRadians(75));

        TrajectoryActionBuilder centerBlockTrajectory = drive.actionBuilder(depositPose)
                .splineToLinearHeading(centerBlockPose, Math.toRadians(100));

        TrajectoryActionBuilder leftBlockTrajectory = drive.actionBuilder(depositPose)
                .splineToLinearHeading(leftBlockPose, Math.toRadians(100));

        TrajectoryActionBuilder depositRightBlockTrajectory = drive.actionBuilder(rightBlockPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, Math.toRadians(225));

        TrajectoryActionBuilder depositCenterBlockTrajectory = drive.actionBuilder(centerBlockPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, Math.toRadians(225));

        TrajectoryActionBuilder depositLeftBlockTrajectory = drive.actionBuilder(leftBlockPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, Math.toRadians(225));

        TrajectoryActionBuilder parkTrajectory = drive.actionBuilder(depositPose)
                .splineToLinearHeading(parkPose, Math.toRadians(0));

        Action depositPreloadApproach = depositPreloadTrajectory.build();
        Action rightBlock = rightBlockTrajectory.build();
        Action centerBlock = centerBlockTrajectory.build();
        Action leftBlock = leftBlockTrajectory.build();
        Action depositRightBlock = depositRightBlockTrajectory.build();
        Action depositCenterBlock = depositCenterBlockTrajectory.build();
        Action depositLeftBlock = depositLeftBlockTrajectory.build();
        Action park = parkTrajectory.build();

        Actions.runBlocking(
                robot.depositor.closeClaw()
        );

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                robot.extension.gotoRetract(),
                                robot.lift.gotoHighBasket(),
                                robot.depositor.gotoUp(),
                                depositPreloadApproach
                        ),
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.25),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.25),

                        new ParallelAction(
                                robot.lift.gotoDeconflict(),
                                rightBlock
                        ),

                        // COLLECT SEQUENCE
                        robot.depositor.gotoDown(),
                        robot.extension.gotoMax(),
                        robot.collector.collectorInAction(),
                        new SleepAction(1.0),
                        robot.collector.collectorOffAction(),
                        robot.extension.gotoRetract(),

                        // DEPOSIT SEQUENCE
                        robot.lift.gotoGrab(),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.35),
                        robot.lift.gotoDeconflict(),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.5),
                        robot.lift.gotoHighBasket(),
                        depositLeftBlock,

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.5),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.75),

                        // CENTER BLOCK
                        new ParallelAction(
                                robot.lift.gotoDeconflict(),
                                centerBlock
                        ),

                        // COLLECT SEQUENCE
                        robot.depositor.gotoDown(),
                        robot.extension.gotoMax(),
                        robot.collector.collectorInAction(),
                        new SleepAction(1.0),
                        robot.collector.collectorOffAction(),
                        robot.extension.gotoRetract(),

                        // DEPOSIT SEQUENCE
                        robot.lift.gotoGrab(),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.35),
                        robot.lift.gotoDeconflict(),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.5),
                        robot.lift.gotoHighBasket(),
                        depositCenterBlock,

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.5),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.75),

                        // LEFT BLOCK
                        new ParallelAction(
                                robot.lift.gotoDeconflict(),
                                leftBlock
                        ),

                        // COLLECT SEQUENCE
                        robot.depositor.gotoDown(),
                        robot.extension.gotoMax(),
                        robot.collector.collectorInAction(),
                        new SleepAction(1.0),
                        robot.collector.collectorOffAction(),
                        robot.extension.gotoRetract(),

                        // DEPOSIT SEQUENCE
                        robot.lift.gotoGrab(),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.35),
                        robot.lift.gotoDeconflict(),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.5),
                        robot.lift.gotoHighBasket(),
                        depositRightBlock,

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.5),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.75),

                        // PARK
                        park
                )
        );

    }
}