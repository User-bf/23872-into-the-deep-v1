
package org.firstinspires.ftc.teamcode.teleop.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftDeconflictCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftGrabCommand;

public class DepositReleaseSequenceCommand extends SequentialCommandGroup {
    public DepositReleaseSequenceCommand(BrainSTEMRobot robot, Telemetry telemetry){
        super(
                new GripperOpenCommand(robot.depositor, telemetry),
                new DepositorBackCommand(robot.depositor, telemetry),
                new LiftDeconflictCommand(robot.lift, telemetry)
        );
    }
}