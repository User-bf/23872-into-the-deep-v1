package org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Depositor;

public class DepositorHighBasketCommand extends CommandBase {
    Depositor depositor;
    Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();



    public DepositorHighBasketCommand(Depositor depositor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.depositor = depositor;
    }

    @Override
    public void initialize() {
        depositor.setDepositorHighBasket();
        timer.reset();
    }

    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > Depositor.Params.DEPOSITOR_HIGH_BASKET_TIME_MS;
    }
}
