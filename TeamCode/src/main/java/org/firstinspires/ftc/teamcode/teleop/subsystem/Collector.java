package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;

@Config
public class Collector implements Component {
    public static class Params {

    }

    Telemetry telemetry;
    HardwareMap hardwareMap;
    CachingMotor collectorMotor;
    public static Params PARAMS = new Params();

    public CollectorState collectorState;

    public Collector(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        collectorState = CollectorState.LEVEL;
        collectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "collector"));
    }

    public enum CollectorState {
        INTAKE,
        EJECT,
        LEVEL

    }

    @Override
    public void reset() {}

    @Override
    public String test() {
        return "true";
    }

    @Override
    public void update() {
        switch (collectorState) {
            case LEVEL: {
                collectorLevel();
                break;
            }

            case INTAKE: {
                collectorIn();
                break;
            }

            case EJECT: {
                collectorOut();
                break;
            }
        }
    }

    public double getPower(){
        return collectorMotor.getPower();
    }
    public CollectorState getState(){
        return collectorState;
    }
    private void collectorLevel() {
        collectorMotor.setPower(0.0);
    }

    private void collectorIn() {
        collectorMotor.setPower(1.0);
    }

    private void collectorOut() {
<<<<<<< HEAD
        collectorMotor.setPower(-1.0);
=======
        collectorMotor.setPower(-0.99);
>>>>>>> 9ab5b07ac9d42dee42c582e5627c9c58eed1026c
    }

    public void setIntake() {
        collectorState = CollectorState.INTAKE;
    }

    public void setEject() {
        collectorState = CollectorState.EJECT;
    }

    public void setLevel() {
        collectorState = CollectorState.LEVEL;
    }

}
