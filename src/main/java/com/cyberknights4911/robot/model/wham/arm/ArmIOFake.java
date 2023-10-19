package com.cyberknights4911.robot.model.wham.arm;

public final class ArmIOFake implements ArmIO {

    @Override
    public void updateInputs(ArmIOInputs inputs) {}

    @Override
    public double getWristEncoderDegrees() {
        return 0;
    }

    @Override
    public double getShoulderEncoderDegrees() {
        return 0;
    }

    @Override
    public void setShoulderPosition(double position) {}

    @Override
    public void setWristPosition(double position) {}

    @Override
    public double getShoulderTrajectoryPosition() {
        return 0;
    }

    @Override
    public double getWristTrajectoryPosition() {
        return 0;
    }
    
}
