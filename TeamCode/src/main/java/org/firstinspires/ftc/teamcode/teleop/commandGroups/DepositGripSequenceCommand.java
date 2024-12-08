
package org.firstinspires.ftc.teamcode.teleop.commandGroups;

import com.arc
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftBaseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftDeconflictCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftGrabCommand;

public class DepositGripSequenceCommand extends SequentialCommandGroup {
    public DepositGripSequenceCommand(BrainSTEMRobot robot, Telemetry telemetry){
        super(
                new GripperOpenCommand(robot.depositor, telemetry),
                new DepositorDownCommand(robot.depositor, telemetry),
                new LiftGrabCommand(robot.lift, telemetry),
                new GripperCloseCommand(robot.depositor, telemetry),
                new LiftDeconflictCommand(robot.lift, telemetry),
                new DepositorBackCommand(robot.depositor, telemetry)
        );
    }
}