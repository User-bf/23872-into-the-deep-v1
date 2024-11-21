package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.PIDDrivetrain;
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

        PIDDrivetrain drive = new PIDDrivetrain(hardwareMap, telemetry, new Pose2D(DistanceUnit.INCH,0,0,
                AngleUnit.RADIANS, 0));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            updateDrive(robot);
            robot.update();

            updateDriver1(robot);
            updateDriver2(robot);
            telemetry.addData("Collector Power", robot.collector.getPower());
            telemetry.addData("Collector State", robot.collector.getState());
            telemetry.update();
        }
    }

    private void updateDriver1(BrainSTEMRobot robot) {
        driver1LiftControls(robot);
        driver1DepositorControls(robot);
        driver1CollectorControls(robot);
        driver1ExtensionControls(robot);
        driver2ExtensionControls(robot);
       // driver2CollectorControls(robot);
        driver2LiftControls(robot);
    }

    private void driver2LiftControls(BrainSTEMRobot robot) {
        if(gamepad2.left_trigger > 0.5) {
            new GrabSpecimenSequenceCommand(robot, telemetry).schedule();
        }
        else if (gamepad2.right_trigger > 0.5) {
            new SpecimenPreDeposit(robot, telemetry).schedule();
        }

        if(gamepad2.y) {
            new DepositSpecimenHighBarSequenceCommand(robot,telemetry).schedule();
        }

    }
    private void driver2ExtensionControls(BrainSTEMRobot robot) {
        if (gamepad1.dpad_up) {
            robot.extension.setCustom();
            robot.extension.incrementOut();
        } else if (gamepad1.dpad_down) {
            robot.extension.setCustom();
            robot.extension.incrementIn();
        }

        if (gamepad1.a) {
            robot.extension.setRetract();
        }

    }

//    private void driver2CollectorControls(BrainSTEMRobot robot) {
//        if (gamepad2.right_trigger > 0) {
//            robot.collector.setIntake();
//        }
//    }

    private void driver1LiftControls(BrainSTEMRobot robot) {
        if (gamepad2.dpad_up) {
            new LiftHighBasketCommand(robot.lift, telemetry).schedule();
        } else if (gamepad2.dpad_down) {
            new LiftLowBasketCommand(robot.lift, telemetry).schedule();
        }
        if (gamepad2.dpad_right)
            new DepositSpecimenHighBarSequenceCommand(robot, telemetry).schedule();
    }

    private void driver1ExtensionControls(BrainSTEMRobot robot) {
    }

    private void driver1CollectorControls(BrainSTEMRobot robot) {
        if (gamepad1.a) {
            robot.collector.setIntake();
        } else if (gamepad1.b) {
            robot.collector.setEject();
        }
        else {
            robot.collector.setLevel();
        }
    }

    private void driver1DepositorControls(BrainSTEMRobot robot) {
//        if (gamepad1.left_bumper) {
//            robot.depositor.setDepositorForward();
//        } else if (gamepad1.right_bumper) {
//            robot.depositor.setDepositorNeutral();
//        } else {
//            robot.depositor.setDepositorBackward();
//        }
//
        if (gamepad2.right_trigger > 0.5) {
            new DepositReleaseSequenceCommand(robot, telemetry).schedule();
        }
//        else {
//            new GripperCloseCommand(robot.depositor, telemetry).schedule();
//        }

        if (gamepad2.right_bumper) {
            new DepositGripSequenceCommand(robot, telemetry).schedule();
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
        drawRobot(robot);
    }
}
