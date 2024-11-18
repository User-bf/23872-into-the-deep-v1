package org.firstinspires.ftc.teamcode.teleop.commandGroups;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftSpecimenPreDeposit;

public class SpecimenPreDeposit extends SequentialCommandGroup {
    public SpecimenPreDeposit(BrainSTEMRobot robot, Telemetry telemetry) {
        super(
                new GripperCloseCommand(robot.depositor, telemetry),
                new LiftSpecimenPreDeposit(robot.lift, telemetry),
                new DepositorForwardCommand(robot.depositor, telemetry)
        );

    }

}