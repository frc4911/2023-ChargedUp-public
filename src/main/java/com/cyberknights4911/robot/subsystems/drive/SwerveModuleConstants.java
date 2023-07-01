package com.cyberknights4911.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveModuleConstants {
    private final int driveMotorId;
    private final int angleMotorId;
    private final int canCoderId;
    private final Rotation2d angleOffset;

    public SwerveModuleConstants(
        int driveMotorId,
        int angleMotorId,
        int canCoderId,
        Rotation2d angleOffset
    ) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.canCoderId = canCoderId;
        this.angleOffset = angleOffset;
    }
    public int getDriveMotorId() {
        return driveMotorId;
    }

    public int getAngleMotorId() {
        return angleMotorId;
    }

    public int getCanCoderId() {
        return canCoderId;
    }

    public Rotation2d getAngleOffset() {
        return angleOffset;
    }
}
