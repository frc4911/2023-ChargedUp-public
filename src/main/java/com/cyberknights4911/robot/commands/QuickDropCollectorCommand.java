package com.cyberknights4911.robot.commands;

import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.collector.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;

public final class QuickDropCollectorCommand extends CommandBase {

    private final Collector collector;
    private final double percentOutput;

    public QuickDropCollectorCommand(Subsystems subsystems, double percentOutput) {
        collector = subsystems.getCollector();
        this.percentOutput = percentOutput;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.run(percentOutput);
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
