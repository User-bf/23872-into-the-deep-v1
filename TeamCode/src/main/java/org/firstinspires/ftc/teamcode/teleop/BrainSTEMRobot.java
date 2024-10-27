package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Component;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Lift;

import java.util.ArrayList;

public class BrainSTEMRobot {
    Telemetry telemetry;
    HardwareMap map;
    ArrayList subsystems;
    public Lift lift;

    public PinpointDrive drive;

    public BrainSTEMRobot(Telemetry telemetry, HardwareMap map){
        this.telemetry = telemetry;
        this.map = map;

        lift = new Lift(map, telemetry);
        drive = new PinpointDrive(map, new Pose2d(0,0,0));
    }

    public void update() {
        lift.update();
    }
}
