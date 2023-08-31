package com.cyberknights4911.robot.model.quickdrop.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

/*
 * One ball
 * if enter is blocked
 *   while exit is not blocked
 *      motor on
 *   motor off
 * 
 */
public class IndexerCommand extends CommandBase {
    
    private final Indexer indexer;
    private final double percentOutput;

    public IndexerCommand(Indexer indexer, double percentOutput) {
        this.indexer = indexer;
        this.percentOutput = percentOutput;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (indexer.isEnterBlocked()) {
            indexer.run(percentOutput);

        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return indexer.isExitBlocked();
    }
}
