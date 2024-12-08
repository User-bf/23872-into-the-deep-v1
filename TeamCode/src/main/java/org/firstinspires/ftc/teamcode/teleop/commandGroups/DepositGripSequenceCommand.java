
package org.firstinspires.ftc.teamcode.teleop.commandGroups;

<<<<<<< HEAD
import com.arc
=======
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

>>>>>>> 9ab5b07ac9d42dee42c582e5627c9c58eed1026c
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorHighBasketCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftBaseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftDeconflictCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftGrabCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftGrabSpecimenCommand;

public class DepositGripSequenceCommand extends SequentialCommandGroup {
    public DepositGripSequenceCommand(BrainSTEMRobot robot, Telemetry telemetry){
        super(
                new GripperOpenCommand(robot.depositor,telemetry),
                new DepositorDownCommand(robot.depositor,telemetry),
                new LiftGrabCommand(robot.lift,telemetry),
                new WaitCommand(500),
                new GripperCloseCommand(robot.depositor,telemetry),
                new WaitCommand(500),
                new LiftDeconflictCommand(robot.lift,telemetry),
                new DepositorHighBasketCommand(robot.depositor,telemetry)
        );
    }
}