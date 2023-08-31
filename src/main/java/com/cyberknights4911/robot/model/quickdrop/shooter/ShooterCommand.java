package com.cyberknights4911.robot.model.quickdrop.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {

    private final Shooter shooter;
    private final double velocityOutput;

    public ShooterCommand(Shooter shooter, double velocityOutput) {
        this.shooter = shooter;
        this.velocityOutput = velocityOutput;
        addRequirements(shooter);
    }
}
