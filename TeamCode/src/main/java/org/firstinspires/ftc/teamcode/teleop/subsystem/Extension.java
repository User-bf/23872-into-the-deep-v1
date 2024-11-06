package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    // PIDS Values
    public double kP_Up = 0.05;//FIXME
    public double kI_Up = 0.00; //FIXME
    public double kD_Up = 0.000;//FIXME
    public double kS= 0;

    // some variables needed for class
    public double target = 0;


    private double power = 0;

    private int error = 0;
    // instantiating PIDController
    PIDController extensionController;

    // Constants
    private static final int EXTENTION_MAX = 20000;
    private static final int EXTENSION_IN = 0;
    private int EXTENSION_CUSTOM = 200;



    // constructor for Extension class
    public Extension(HardwareMap hwMap, Telemetry telemetry) {

        extensionController = new PIDController(kP_Up, kI_Up, kD_Up);
        this.telemetry = telemetry;
        this.map = hwMap;

        extensionController.setInputBounds(EXTENSION_IN, EXTENTION_MAX);
        extensionController.setOutputBounds(-1.0, 1.0);
        extension = new CachingMotor(map.get(DcMotorEx.class, "extension"));
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // creating enums for Extension
    public enum ExtensionState {
        OUT, IN, OFF, CUSTOM
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
            power = extensionController.updateWithError(error) + kS;
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
        extensionState = extensionState.IN;
    }

    public void setCustom(){extensionState = ExtensionState.CUSTOM;}


    public void incrementOut(){
        EXTENSION_CUSTOM += 20;
    }

    public void incrementIn(){
        EXTENSION_CUSTOM -= 20;
    }

    public void setTelemetry() {}

    // placeholder function
    @Override
    public void reset() {

    }

    // update function for setting the state to extension
    @Override
    public void update() {
        switch (extensionState) {
            case OUT:
                extension.setPower(1.0);
                break;

            case IN:
                extension.setPower(-1.0);
                break;

            case OFF:
                setExtensionPower(0);
                break;

            case CUSTOM:
                setExtensionPower(EXTENSION_CUSTOM);
                break;
        }

        telemetry.addData("Extension Position", extension.getCurrentPosition());
        telemetry.addData("Extension Power", extension.getPower());
    }

    public String test() {
        return null;
    }

}
