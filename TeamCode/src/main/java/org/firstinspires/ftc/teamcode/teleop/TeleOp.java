package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.PIDDrivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositGripSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositReleaseSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositSpecimenHighBarSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.GrabSpecimenSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.SpecimenPreDeposit;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftHighBasketCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftLowBasketCommand;
import org.firstinspires.ftc.teamcode.util.Drawing;

public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            updateDrive(robot);
            updateDriver1(robot);
            telemetry.update();
        }
    }


    private void updateDriver1(BrainSTEMRobot robot) {
        driver2LiftControls(robot);
        driver2DepositorControls(robot);
        driver1CollectorControls(robot);
        driver1ExtensionControls(robot);
        driver2DepositorControls(robot);
    }

    private void driver1ExtensionControls(BrainSTEMRobot robot) {
        if (gamepad1.dpad_up) {
            robot.extension.incrementOut();
            robot.extension.setCustom();
        } else if (gamepad1.dpad_down) {
            robot.extension.setCustom();
            robot.extension.incrementIn();
        }

        if (gamepad1.x) {
            robot.extension.setRetract();
        }
    }

<<<<<<< HEAD
    private void driver2CollectorControls(BrainSTEMRobot robot) {
        if (gamepad2.right_trigger > 0) {
            robot.collector.setIntake();
        } else {
            robot.collector.setLevel();
        }
    }

    private void driver1LiftControls(BrainSTEMRobot robot) {
        if (gamepad1.dpad_up) {
            new LiftHighBasketCommand(robot.lift, telemetry);
        } else if (gamepad1.dpad_down) {
            new LiftLowBasketCommand(robot.lift, telemetry);
=======
    private void driver2LiftControls(BrainSTEMRobot robot) {
        if (gamepad2.dpad_up) {
            new LiftHighBasketCommand(robot.lift, telemetry).schedule();
        } else if (gamepad2.dpad_down) {
            new LiftLowBasketCommand(robot.lift, telemetry).schedule();
>>>>>>> 9ab5b07ac9d42dee42c582e5627c9c58eed1026c
        }
        if (gamepad2.y) {
            new DepositSpecimenHighBarSequenceCommand(robot, telemetry).schedule();
        }
    }

    private void driver1CollectorControls(BrainSTEMRobot robot){

        if (gamepad1.a) {
            robot.collector.setIntake();
        } else if (gamepad1.b) {
            robot.collector.setEject();
        }
        else {
            robot.collector.setLevel();
        }
    }

<<<<<<< HEAD
    private void driver1DepositorControls(BrainSTEMRobot robot) {
//        if (gamepad1.left_bumper) {
//            robot.depositor.setDepositorForward();
//        } else if (gamepad1.right_bumper) {
//            robot.depositor.setDepositorNeutral();
//        } else {
//            robot.depositor.setDepositorBackward();
//        }
//
        if (gamepad1.right_trigger > 0.5) {
            new DepositReleaseSequenceCommand(robot, telemetry);
=======
    private void driver2DepositorControls(BrainSTEMRobot robot) {
        if (gamepad2.left_bumper) {
            new DepositReleaseSequenceCommand(robot, telemetry).schedule();
>>>>>>> 9ab5b07ac9d42dee42c582e5627c9c58eed1026c
        }

<<<<<<< HEAD
        if (gamepad1.right_bumper) {
            new DepositGripSequenceCommand(robot, telemetry);
=======
        if (gamepad2.right_bumper) {
            new DepositGripSequenceCommand(robot, telemetry).schedule();
>>>>>>> 9ab5b07ac9d42dee42c582e5627c9c58eed1026c
        }
        if (gamepad2.left_trigger > 0.5) {
            new GrabSpecimenSequenceCommand(robot, telemetry).schedule();
        } else if (gamepad2.right_trigger > 0.5) {
            new SpecimenPreDeposit(robot, telemetry).schedule();

        }
    }

    private void updateDriver2(BrainSTEMRobot robot) {
    }

    private void drawRobot(BrainSTEMRobot robot) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), robot.drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void updateDrive(BrainSTEMRobot robot) {
        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        telemetry.addData("x", robot.drive.pose.position.x);
        telemetry.addData("y", robot.drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.pose.heading.toDouble()));
        telemetry.addData("Extension Pow", robot.extension.extension.getPower());

        drawRobot(robot);
    }
}
