package com.cyberknights4911.robot.model.quickdrop.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;

public final class QuickDropCollectorCommand extends CommandBase {

    private final Collector collector;
    private final double percentOutput;

    public QuickDropCollectorCommand(Collector collector, double percentOutput) {
        this.collector = collector;
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
