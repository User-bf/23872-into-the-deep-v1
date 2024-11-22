package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoDrivetrain extends PinpointDrive {

    public AutoDrivetrain(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
    }
}
