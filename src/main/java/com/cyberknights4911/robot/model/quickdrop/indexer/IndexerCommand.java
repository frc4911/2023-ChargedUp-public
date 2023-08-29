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
        System.out.println("Indexer Initializing...");
    }

    @Override
    public void execute() {
        System.out.println("Execute Executing...");
        System.out.println("isBlocked = " + indexer.isEnterBlocked());
        if (indexer.isEnterBlocked()) {
            System.out.println("Setting output: " + percentOutput);
            indexer.run(percentOutput);

        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        System.out.println("isFinished = " + indexer.isExitBlocked());
        return indexer.isExitBlocked();
    }
}
