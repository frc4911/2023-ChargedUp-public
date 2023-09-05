package com.cyberknights4911.robot.drive.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class TeleopSwerveCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier translationSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldCentricSupplier;
    private final double deadband;
    private final double maxSpeed;
    private final double maxAngularVelocity;

    public TeleopSwerveCommand(
        SwerveSubsystem swerveSubsystem,
        DoubleSupplier translationSupplier,
        DoubleSupplier strafeSupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier fieldCentricSupplier,
        double maxSpeed,
        double maxAngularVelocity,
        double deadband
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;
        this.deadband = deadband;
        this.maxSpeed = maxSpeed;
        this.maxAngularVelocity = maxAngularVelocity;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        /* Get values, applying deadband */
        double translationVal = MathUtil.applyDeadband(translationSupplier.getAsDouble(), deadband);
        double strafeVal = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), deadband);
        double rotationVal = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), deadband);

        /* Drive the robot */
        swerveSubsystem.drive(
            new Translation2d(translationVal, strafeVal).times(maxSpeed),
            rotationVal * maxAngularVelocity,
            fieldCentricSupplier.getAsBoolean(),
            true
        );
    }
}