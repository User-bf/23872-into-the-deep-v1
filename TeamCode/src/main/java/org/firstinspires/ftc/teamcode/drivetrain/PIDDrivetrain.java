package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.PathPoint;

import java.util.ArrayList;

@Config
public class PIDDrivetrain {
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private Pose2D lastPinpointPose;
    private Pose2D currentPose;
    private Pose2D targetPose;
    private PIDController forwardPIDControl;
    private PIDController strafePIDControl;
    private PIDController headPIDControl;
    private Telemetry telemetry;

    private double calculatedLeftFrontPower, calculatedLeftBackPower, calculatedRightFrontPower,
            calculatedRightBackPower;

    public static class Params {
        public double forward_kP = 0.125;
        public double forward_kI = 0.0025;
        public double forward_kD = 0.01;

        public double strafe_kP = 0.025;
        public double strafe_kI = 0.0025;
        public double strafe_kD = 0.005;

        public double head_kP = 0.025;
        public double head_kI = 0.001;
        public double head_kD = 0.0;


        public double xOffset = 4.724;
        public double yOffset = 5.197;
        public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public static Params PARAMS = new Params();
    public GoBildaPinpointDriver pinpoint;

    private ArrayList<PathPoint> pathPoints;
    private PathPoint currentPathPoint;

    public PIDDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, Pose2D pose) {
        currentPose = pose;
        this.telemetry = telemetry;
        pathPoints = new ArrayList<PathPoint>();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(PARAMS.yOffset));
        pinpoint.setEncoderResolution(PARAMS.encoderResolution);
        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
        resetPosition(currentPose);

        forwardPIDControl = new PIDController(PARAMS.forward_kP, PARAMS.forward_kI, PARAMS.forward_kD);
        forwardPIDControl.setInputBounds(-72, 72);
        forwardPIDControl.setOutputBounds(-1.0,1.0);

        strafePIDControl = new PIDController(PARAMS.strafe_kP, PARAMS.strafe_kI, PARAMS.strafe_kD);
        strafePIDControl.setInputBounds(-72, 72);
        forwardPIDControl.setOutputBounds(-1.0,1.0);

        headPIDControl = new PIDController(PARAMS.head_kP, PARAMS.head_kI, PARAMS.head_kD);
        headPIDControl.setInputBounds(-360, 360);
        headPIDControl.setOutputBounds(-1.0, 1.0);



        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftBack = hardwareMap.get(DcMotorEx.class, "BL");
        rightBack = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setTargetPose(Pose2D targetPose) {
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
        return pinpoint.getPosition();
    }

    public Pose2D getCurrentPose() { return currentPose; }

    public Pose2D getTargetPose() { return targetPose; }

    private void moveToTargetPose() {
        forwardPIDControl.setTarget(targetPose.getX(DistanceUnit.INCH));
        strafePIDControl.setTarget(targetPose.getY(DistanceUnit.INCH));
        headPIDControl.setTarget(targetPose.getHeading(AngleUnit.DEGREES));
        calculateMotorPowers();

        setDrivePower(calculatedLeftFrontPower, calculatedLeftBackPower, calculatedRightFrontPower,
                calculatedRightBackPower);
    }

    public void addPathPoint(Pose2D pathPoint, double displacementTolerance, double headingTolerance) {
        pathPoints.add(new PathPoint(pathPoint, displacementTolerance, headingTolerance));
    }

    public void drivePath() {
        updatePoseEstimate();
        PathPoint firstPathPoint = pathPoints.get(0);
        if (firstPathPoint.inTolerance(currentPose)) {
            pathPoints.remove(0);
        }
        currentPathPoint = pathPoints.get(0);

        setTargetPose(currentPathPoint.getPoint());
        moveToTargetPose();
    }

    private void calculateMotorPowers() {
        double forwardPower = -forwardPIDControl.update(currentPose.getX(DistanceUnit.INCH));
        double strafePower =  strafePIDControl.update(currentPose.getY(DistanceUnit.INCH));
        double headPower = headPIDControl.update(currentPose.getHeading(AngleUnit.DEGREES));

        headPIDControl.getTarget();
        telemetry.addData("headPIDControl target", headPIDControl.getTarget());
        telemetry.addData("headPower", headPower);
        telemetry.addData("target heading", getTargetPose().getHeading(AngleUnit.DEGREES));
        telemetry.addData("current heading", getCurrentPose().getHeading(AngleUnit.DEGREES));

        calculatedLeftFrontPower = forwardPower + strafePower + headPower;
        calculatedLeftBackPower = forwardPower - strafePower + headPower;
        calculatedRightFrontPower = forwardPower - strafePower - headPower;
        calculatedRightBackPower = forwardPower + strafePower - headPower;

//        telemetry.addData("forwardPower", forwardPower);
//        telemetry.addData("strafePower", strafePower);
//        telemetry.addData("headPower", headPower);
//        telemetry.addData("calculatedLeftFrontPower", calculatedLeftFrontPower);
//        telemetry.addData("calculatedLeftBackPower", calculatedLeftBackPower);
//        telemetry.addData("calculatedRightFrontPower", calculatedRightFrontPower);
//        telemetry.addData("calculatedRightBackPower", calculatedRightBackPower);
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
