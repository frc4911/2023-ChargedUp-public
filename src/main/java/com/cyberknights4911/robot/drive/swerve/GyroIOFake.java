package com.cyberknights4911.robot.drive.swerve;

final class GyroIOFake implements GyroIO {

    @Override
    public void updateInputs(GyroIOInputs inputs) {}

    @Override
    public double getYaw() {
        return 0;
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }

    @Override
    public void setYaw(double angleInDegrees) {}
    
}
