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
        Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 30, 0, AngleUnit.DEGREES, 0);

        PIDDrivetrain drive = new PIDDrivetrain(hardwareMap, telemetry, initialPose);
        drive.resetPosition(initialPose);

        drive.addPathPoint(targetPose, 1.0, 2);
        drive.addPathPoint(initialPose, 1.0, 2);
        drive.addPathPoint(targetPose, 1.0, 2);
        drive.addPathPoint(initialPose, 1.0, 2);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.drivePath();
        }
    }
}
