package com.cyberknights4911.robot.commands;

import java.util.function.DoubleSupplier;

import com.cyberknights4911.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultSwerveCommand extends CommandBase {
 
    private final SwerveSubsystem mSwerve;

    private final DoubleSupplier mTranslationXSupplier;
    private final DoubleSupplier mTranslationYSupplier;
    private final DoubleSupplier mRotationSupplier;

    public DefaultSwerveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.mSwerve = swerveSubsystem;
        this.mTranslationXSupplier = translationXSupplier;
        this.mTranslationYSupplier = translationYSupplier;
        this.mRotationSupplier = rotationSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        mSwerve.setTeleopInputs(
                mTranslationXSupplier.getAsDouble(),
                mTranslationYSupplier.getAsDouble(),
                mRotationSupplier.getAsDouble(),
                false,
                true,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.setTeleopInputs(
                0.0,
                0.0,
                0.0,
                false,
                true,
                true);
    }
}