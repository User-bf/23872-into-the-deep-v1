package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

public class PIDDrivetrain {
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private Pose2D lastPinpointPose;
    public Pose2D currentPose;
    private Pose2D targetPose;

    public static class Params {
        public double xOffset = 119.9896;
        public double yOffset = 132.0038;
        public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public static PinpointDrive.Params PARAMS = new PinpointDrive.Params();
    public GoBildaPinpointDriver pinpoint;


    public PIDDrivetrain(HardwareMap hardwareMap, Pose2D pose) {
        currentPose = pose;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(PARAMS.yOffset));
        pinpoint.setEncoderResolution(PARAMS.encoderResolution);
        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);

        pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(pose);

        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftBack = hardwareMap.get(DcMotorEx.class, "BL");
        rightBack = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTargetPose(Pose2D targetPose) {
        this.targetPose = targetPose;
    }

    private Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public Pose2D updatePoseEstimate() {
        if (lastPinpointPose != currentPose) {
            pinpoint.setPosition(currentPose);
        }
        pinpoint.update();
        currentPose = getPose();
        lastPinpointPose = currentPose;

        return pinpoint.getVelocity();
    }

    public void setDrivePower(double leftFrontPower, double leftBackPower, double rightFrontPower,
                              double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

}
