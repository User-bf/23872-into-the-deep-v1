package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class Extension implements Component {

    // initialization
    private Telemetry telemetry;
    public DcMotorEx extension;
    private HardwareMap map;

    private AnalogInput eTrackerEncoder;

    public static class Params {
        // PIDS Values
        public double kP_Up = 0.05;//FIXME
        public double kI_Up = 0.00; //FIXME
        public double kD_Up = 0.000;//FIXME
        public double kS = 0;

        public int TOLERANCE = 5;

        public static final int EXTENSION_MAX = 500;
        public static final int EXTENSION_IN = 0;
        public int EXTENSION_CUSTOM = 200;
        public static final int RETRACT_POSITION = 0;
    }

    // some variables needed for class
    public double target = 0;


    private double power = 0;

    private int error = 0;
    // instantiating PIDController
    PIDController extensionController;
    DigitalChannel extensionLimitSwitch;

    // Constants

    public static Extension.Params PARAMS = new Extension.Params();


    // constructor for Extension class
    public Extension(HardwareMap hwMap, Telemetry telemetry) {

        extensionController = new PIDController(PARAMS.kP_Up, PARAMS.kI_Up, PARAMS.kD_Up);
        this.telemetry = telemetry;
        this.map = hwMap;

        extensionController.setInputBounds(PARAMS.EXTENSION_IN, PARAMS.EXTENSION_MAX);
        extensionController.setOutputBounds(-1.0, 1.0);
        extension = new CachingMotor(map.get(DcMotorEx.class, "extension"));
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionLimitSwitch = hwMap.get(DigitalChannel.class, "eLimitSwitch");
    }

    // creating enums for Extension
    public enum ExtensionState {
        OUT,
        IN,
        OFF,
        CUSTOM,
        RETRACT
    }
    // creating extensionState var
    public ExtensionState extensionState = ExtensionState.IN;

    // setting the extension power off
    public void extensionOff(){
        extension.setPower(0);
    }
    // setting the pid extension power customizable
    private void setExtensionPower(int ticks){
        target = ticks;
        error = extension.getCurrentPosition() - ticks;
        if(!(Math.abs(error) <= 500)) {
            power = extensionController.updateWithError(error) + PARAMS.kS;
        }
        else {
            power = 0;
        }
        extension.setPower(-power);
    }


//     setting the extension state to off
    public void setOff(){
        extensionState = extensionState.OFF;
    }

    public void setOut(){
        extensionState = extensionState.OUT;
    }

    public void setIn(){
        extensionState = extensionState.RETRACT;
    }

    public void setCustom(){extensionState = ExtensionState.CUSTOM;}


    public void incrementOut(){
        PARAMS.EXTENSION_CUSTOM += 20;
    }

    public void incrementIn(){
        PARAMS.EXTENSION_CUSTOM -= 20;
    }

    public void setTelemetry() {}

    public boolean inTolerance() {
        return Math.abs(extension.getCurrentPosition() - extensionController.getTarget()) < PARAMS.TOLERANCE;
    }

    // placeholder function
    @Override
    public void reset() {

    }

    public boolean isExtensionLimit() {
        return !extensionLimitSwitch.getState();
    }

    private void setTarget(double target) {
        extensionController.setTarget(target);
    }

    private void selectState() {
        switch (extensionState) {
            case OUT:
                setTarget(PARAMS.EXTENSION_MAX);
                break;

            case RETRACT:
                if (isExtensionLimit()) {
                    extension.setPower(0);
                    extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else {
                    setTarget(PARAMS.RETRACT_POSITION);
                }
                break;

            case OFF:
                setExtensionPower(0);
                break;

            case CUSTOM:
                setExtensionPower(PARAMS.EXTENSION_CUSTOM);
                break;
        }
    }

    private double getControlPower() {
        double pidPower = -extensionController.update(extension.getCurrentPosition());

        return pidPower;
    }

    public void setMotorPower(double power) {
        extension.setPower(power);
    }

    // update function for setting the state to extension
    @Override
    public void update() {
        selectState();
        setMotorPower(getControlPower());
        telemetry.addData("Extension Position", extension.getCurrentPosition());
        telemetry.addData("Extension Power", extension.getPower());
        telemetry.addData("Extension Limit", isExtensionLimit());
    }

    public String test() {
        return null;
    }

}
