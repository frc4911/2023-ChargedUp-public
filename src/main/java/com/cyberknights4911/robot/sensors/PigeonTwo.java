package com.cyberknights4911.robot.sensors;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonTwo implements IMU{
        private static PigeonTwo instance = null;

    private Pigeon2 pigeon;

    private PigeonTwo() {
        try {
            pigeon = new Pigeon2(Ports.PIGEON, Constants.CANIVORE_NAME);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public static PigeonTwo getInstance() {
        if (instance == null) {
            instance = new PigeonTwo();
        }
        return instance;
    }

    @Override
    public boolean isGood() {
        // Unlike the original Pigeon, the Pigeon2 is always ready
        return true;
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
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
    public double[] getYPR() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr;
    }

    @Override
    public void setAngle(double angleInDegrees) {
        pigeon.setYaw(-angleInDegrees, Constants.LONG_CAN_TIMEOUTS_MS);
        System.out.println("Pigeon angle set to: " + angleInDegrees);
    }

    @Override
    public void outputToSmartDashboard() {
        // no-op
    }
}
