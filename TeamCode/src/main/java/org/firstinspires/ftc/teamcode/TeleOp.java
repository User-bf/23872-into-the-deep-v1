package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Lift;

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

            telemetry.update();
        }
    }

    private void updateDriver1(BrainSTEMRobot robot) {
        driver1LiftControls(robot);
        driver1DepositorControls(robot);
        driver1CollectorControls(robot);
        driver1ExtensionControls(robot);
    }

    private void driver1LiftControls(BrainSTEMRobot robot) {
        if (gamepad1.dpad_up) {
            robot.lift.setLevel2();
        } else if (gamepad1.dpad_down) {
            robot.lift.setLevel1();
        }
    }

    private void driver1ExtensionControls(BrainSTEMRobot robot) {
        if (gamepad1.x) {
            robot.extension.setOut();
        } else if (gamepad1.y) {
            robot.extension.setIn();
        } else {
            robot.extension.setOff();
        }
    }

    private void driver1CollectorControls(BrainSTEMRobot robot) {
        if (gamepad1.a) {
            robot.collector.setIntake();
        } else if (gamepad1.b) {
            robot.collector.setEject();
        } else {
            robot.collector.setLevel();
        }
    }

    private void driver1DepositorControls(BrainSTEMRobot robot) {
        if (gamepad1.left_bumper) {
            robot.depositor.setDepositorForward();
        } else {
            robot.depositor.setDepositorBackward();
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
