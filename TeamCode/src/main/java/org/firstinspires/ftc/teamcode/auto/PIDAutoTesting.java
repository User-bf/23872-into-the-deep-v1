package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.drivetrain.PIDDrivetrain;

@Autonomous(name="PID Auto Testing", group="Hippos")
public class PIDAutoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 20, 20, AngleUnit.DEGREES, 45);

        PIDDrivetrain drive = new PIDDrivetrain(hardwareMap, telemetry, initialPose);
        drive.resetPosition(initialPose);

        drive.addPathPoint(targetPose, 0.5, 2);
        drive.addPathPoint(initialPose, 0.5, 2);
        drive.addPathPoint(targetPose, 0.5, 2);
        drive.addPathPoint(initialPose, 0.5, 2);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.drivePath();
            telemetry.addData("Target X", drive.getTargetPose().getX(DistanceUnit.INCH));
            telemetry.addData("Current X", drive.getCurrentPose().getX(DistanceUnit.INCH));
            telemetry.addData("Target Y", drive.getTargetPose().getY(DistanceUnit.INCH));
            telemetry.addData("Current Y", drive.getCurrentPose().getY(DistanceUnit.INCH));
            telemetry.addData("Target Heading", drive.getTargetPose().getHeading(AngleUnit.DEGREES));
            telemetry.addData("Current Heading", drive.getCurrentPose().getHeading(AngleUnit.DEGREES));
            telemetry.update();

            drawField(drive);
        }
    }

    private void drawField(PIDDrivetrain drive) {
        double x = drive.getCurrentPose().getX(DistanceUnit.INCH);
        double y = drive.getCurrentPose().getY(DistanceUnit.INCH);
        double heading = drive.getCurrentPose().getHeading(AngleUnit.DEGREES);
        Pose2d rrPose = new Pose2d(x, y, Math.toRadians(heading));
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), rrPose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
