package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Lift implements Component {
    public static class Params {
        public double liftKp = 0.2;
        public double liftKi = 0.1;
        public double liftKd = 0.0;

        public int LEVEL_1_HEIGHT = 0;
        public int LEVEL_2_HEIGHT = 500;
        public int LEVEL_3_HEIGHT = 1000;
    }

    PIDController liftController;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public static Params PARAMS = new Params();
    DcMotorEx liftMotor;
    public LiftState liftState;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        liftController = new PIDController(PARAMS.liftKp, PARAMS.liftKi, PARAMS.liftKd);
        liftController.setInputBounds(0,4000);
        liftController.setOutputBounds(-1.0,1.0);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        liftState = LiftState.LEVEL_1;
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        lEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "liftEncoder"));
    }

    public enum LiftState {
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        RESET
    }

    public void setMotorPower(double power) {
        liftMotor.setPower(power);
        telemetry.addData("liftMotor Power", power);
    }

    @Override
    public void reset() {
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setTarget(double target) {
        liftController.setTarget(target);
    }

    private void selectState() {
        switch(liftState) {
            case RESET: {
                reset();
                break;
            }

            case LEVEL_1: {
                setTarget(PARAMS.LEVEL_1_HEIGHT);
                break;
            }

            case LEVEL_2: {
                setTarget(PARAMS.LEVEL_2_HEIGHT);
                break;
            }

            case LEVEL_3: {
                setTarget(PARAMS.LEVEL_3_HEIGHT);
                break;
            }
        }
    }

    private double getControlPower() {
        double pidPower = -liftController.update(liftMotor.getCurrentPosition());

        return pidPower + feedForwardPower();
    }

    private double feedForwardPower() {
        double x = liftMotor.getCurrentPosition();
        double m = -0.0;
        double b = -0.0;

        return m * x + b;
    }

    @Override
    public void update() {
        telemetry.addData("liftController Target", liftController.getTarget());
        telemetry.addData("liftMotor Position", liftMotor.getCurrentPosition());

        selectState();
        setMotorPower(getControlPower());
    }

    @Override
    public String test() {
       return "Complete";
    }
}
