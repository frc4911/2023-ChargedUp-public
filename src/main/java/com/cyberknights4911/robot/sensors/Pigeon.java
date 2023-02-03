package com.cyberknights4911.robot.sensors;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon implements IMU {
    
    private static Pigeon instance = null;

    private PigeonIMU pigeon;

    private Pigeon() {
        try {
            pigeon = new PigeonIMU(Ports.PIGEON);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public static Pigeon getInstance() {
        if (instance == null) {
            instance = new Pigeon();
        }
        return instance;
    }

    @Override
    public boolean isGood() {
        return (pigeon.getState() == PigeonState.Ready) ? true : false;
    }

    @Override
    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        // brian temp debug please move to somewhere better
        SmartDashboard.putNumber("Pigeon Heading", pigeon.getFusedHeading(fusionStatus));
        return Rotation2d.fromDegrees(pigeon.getFusedHeading(fusionStatus)/*-ypr[0]*/);
    }

    @Override
    public double getPitch() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[1];
    }

    @Override
    public double getRoll() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[2];
    }

    @Override
    public double[] getYPR() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr;
    }

    @Override
    public void setAngle(double angleInDegrees) {
        pigeon.setFusedHeading(-angleInDegrees * 64.0, Constants.LONG_CAN_TIMEOUTS_MS);
        pigeon.setYaw(-angleInDegrees, Constants.LONG_CAN_TIMEOUTS_MS);
        System.out.println("Pigeon angle set to: " + angleInDegrees);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Pigeon Good", pigeon.getState().toString());
    }
}
