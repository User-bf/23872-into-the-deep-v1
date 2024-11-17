package org.firstinspires.ftc.teamcode.teleop.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftBaseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftDepositSpecimenCommand;

    public class DepositSpecimenHighBarSequenceCommand extends SequentialCommandGroup {
        public DepositSpecimenHighBarSequenceCommand(BrainSTEMRobot robot, Telemetry telemetry) {
            super(
                    new LiftDepositSpecimenCommand(robot.lift, telemetry),
                    new GripperOpenCommand(robot.depositor, telemetry),
                    new LiftBaseCommand(robot.lift, telemetry),
                    new DepositorBackCommand(robot.depositor,telemetry)
            );

        }
    }
