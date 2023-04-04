package com.cyberknights4911.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.util.Units;

public final class GyroIOReal implements GyroIO {

    private final Pigeon2 pigeon;
    private final double[] xyzDps = new double[3];

    public GyroIOReal() {
        pigeon = new Pigeon2(Ports.Drive.PIGEON);
    }
  
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        pigeon.getRawGyro(xyzDps);
        inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
        inputs.positionRad = Units.degreesToRadians(pigeon.getYaw());
        inputs.velocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
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
        pigeon.setYaw(-angleInDegrees, Constants.LONG_CAN_TIMEOUTS_MS);
        System.out.println("Pigeon yaw set to: " + angleInDegrees);
    }
}
