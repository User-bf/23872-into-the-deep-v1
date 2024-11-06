package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingServo;

@Config
public class Depositor implements Component {
    public static class Params {
        double depositorForwardPosition = 0.4;
        double depositorBackwardPosition = 0.6;
    }

    Telemetry telemetry;
    HardwareMap hardwareMap;
    public static Params PARAMS = new Params();
    CachingServo rotationServo;

    public DepositorState depositorState;

    public Depositor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        rotationServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "depositorRotationServo"));
        depositorState = DepositorState.DEPOSITOR_FORWARD;
    }

    public enum DepositorState {
        DEPOSITOR_FORWARD,
        DEPOSITOR_BACKWARD
    }

    @Override
    public void reset() {}

    @Override
    public String test() {
        return "true";
    }

    @Override
    public void update() {
        switch (depositorState) {
            case DEPOSITOR_FORWARD: {
                moveDepositorForward();
                break;
            }

            case DEPOSITOR_BACKWARD: {
                moveDepositorBackward();
                break;
            }
        }
    }

    private void moveDepositorForward() {
        rotationServo.setPosition(PARAMS.depositorForwardPosition);
    }

    private void moveDepositorBackward() {
        rotationServo.setPosition(PARAMS.depositorBackwardPosition);
    }

    public void setDepositorForward() {
        depositorState = DepositorState.DEPOSITOR_FORWARD;
    }

    public void setDepositorBackward() {
        depositorState = DepositorState.DEPOSITOR_BACKWARD;
    }

}
