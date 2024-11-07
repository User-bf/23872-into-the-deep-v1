package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingServo;

@Config
public class Depositor implements Component {
    public static class Params {
        double depositorForwardPosition = 0.99;
        double depositorBackwardPosition = 0.01;
        double gripperClosedPosition = 0.01;
        double gripperOpenedPosition = 0.99;
    }

    Telemetry telemetry;
    HardwareMap hardwareMap;
    public static Params PARAMS = new Params();
    CachingServo rotationServo;
    CachingServo gripperServo;


    public DepositorState depositorState;
    public GripperState gripperState;

    public Depositor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        rotationServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "depositorRotationServo"));
        gripperServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "gripperServo"));
        rotationServo.setPwmRange(new PwmControl.PwmRange(600, 2400));
        gripperServo.setPwmRange(new PwmControl.PwmRange(1200, 1800));
        depositorState = DepositorState.DEPOSITOR_FORWARD;
        gripperState = GripperState.GRIPPER_OPEN;
    }

    public enum GripperState {
        GRIPPER_CLOSED,
        GRIPPER_OPEN
    }

    public enum DepositorState {
        DEPOSITOR_FORWARD,
        DEPOSITOR_BACKWARD
    }

    @Override
    public void reset() {
    }

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

        switch (gripperState) {
            case GRIPPER_CLOSED: {
                closeGripper();
                break;
            }
            case GRIPPER_OPEN: {
                openGripper();
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

    private void closeGripper() {
        gripperServo.setPosition(PARAMS.gripperClosedPosition);
    }

    private void openGripper() {
        gripperServo.setPosition(PARAMS.gripperOpenedPosition);
    }

    public void setGripperClosed() {
        gripperState = GripperState.GRIPPER_CLOSED;
    }

    public void setGripperOpen() {
        gripperState = GripperState.GRIPPER_OPEN;
    }
}

