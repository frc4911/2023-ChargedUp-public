package com.cyberknights4911.robot.drive.swerve;

import java.util.function.Supplier;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.BasePigeon;
import edu.wpi.first.math.util.Units;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.PigeonFactory;

public final class GyroIORealPigeon implements GyroIO {
    private final CtreError ctreError;
    private final BasePigeon pigeon;

    public GyroIORealPigeon(
        PigeonFactory pigeonFactory, SwerveDriveConstants swerveDriveConstants, CtreError ctreError
    ) {
        this.ctreError = ctreError;
        pigeon = pigeonFactory.create(swerveDriveConstants.gyroId());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
        inputs.positionRad = Units.degreesToRadians(pigeon.getYaw());
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw();
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll();
    }

    @Override
    public void setYaw(double angleInDegrees) {
        ctreError.checkError(pigeon.setYaw(-angleInDegrees, ctreError.canTimeoutMs()));
    }
}
