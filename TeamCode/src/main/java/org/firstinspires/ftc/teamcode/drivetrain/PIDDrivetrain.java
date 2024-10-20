package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class PIDDrivetrain {
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private Pose2D lastPinpointPose;
    private Pose2D currentPose;
    private Pose2D targetPose;
    private PIDController forwardPIDControl;
    private Telemetry telemetry;

    private double calculatedLeftFrontPower, calculatedLeftBackPower, calculatedRightFrontPower,
            calculatedRightBackPower;

    public static class Params {
        public double kP = 0.1;
        public double kI = 0.0;
        public double kD = 0.0;
        public double xOffset = 119.9896;
        public double yOffset = 132.0038;
        public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public static Params PARAMS = new Params();
    public GoBildaPinpointDriver pinpoint;


    public PIDDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, Pose2D pose) {
        currentPose = pose;
        this.telemetry = telemetry;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(PARAMS.yOffset));
        pinpoint.setEncoderResolution(PARAMS.encoderResolution);
        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
        resetPosition(currentPose);

        forwardPIDControl = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);
        forwardPIDControl.setInputBounds(-72, 72);
        forwardPIDControl.setOutputBounds(0,0.2);

        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftBack = hardwareMap.get(DcMotorEx.class, "BL");
        rightBack = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setCurrentPose(Pose2D currentPose) {
        this.currentPose = currentPose;
    }

    public void setTargetPose(Pose2D targetPose) {
        this.targetPose = targetPose;
    }

    public void resetPosition(Pose2D pose) {
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint.setPosition(pose);
    }

    private Pose2D queryPose() {
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Pinpoint Position X", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Position Y", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading", pose.getHeading(AngleUnit.DEGREES));
        return pose;
    }

    public Pose2D getPose() { return currentPose; }

    public void moveToTargetPose() {
        telemetry.addData("currentPoseX", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("targetPoseX", targetPose.getX(DistanceUnit.INCH));
        forwardPIDControl.setTarget(targetPose.getX(DistanceUnit.INCH));
        calculateMotorPowers();

        setDrivePower(calculatedLeftFrontPower, calculatedLeftBackPower, calculatedRightFrontPower,
                calculatedRightBackPower);
    }

    private void calculateMotorPowers() {
        double forwardPower = -forwardPIDControl.update(currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("forwardPower", forwardPower);
        calculatedLeftFrontPower = forwardPower;
        calculatedLeftBackPower = forwardPower;
        calculatedRightFrontPower = forwardPower;
        calculatedRightBackPower = forwardPower;
    }

    public Pose2D updatePoseEstimate() {
        if (lastPinpointPose != currentPose) {
            pinpoint.setPosition(currentPose);
        }
        pinpoint.update();
        currentPose = queryPose();
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
