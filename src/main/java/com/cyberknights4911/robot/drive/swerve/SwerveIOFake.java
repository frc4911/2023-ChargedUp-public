package com.cyberknights4911.robot.drive.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

final class SwerveIOFake implements SwerveIO {

    @Override
    public void updateInputs(SwerveIOInputs inputs) {}

    @Override
    public void resetToAbsolute() {}

    @Override
    public void setAngle(ControlMode mode, double outputValue) {}

    @Override
    public void setDrive(ControlMode mode, double outputValue) {}

    @Override
    public void setDrive(ControlMode mode, double outputValue, DemandType demandType, double demandValue) {}

    @Override
    public double getDriveSensorPosition() {
        return 0;
    }

    @Override
    public double getAngleSensorPosition() {
        return 0;
    }

    @Override
    public double getDriveSensorVelocity() {
        return 0;
    }

    @Override
    public double getAngleEncoderDegrees() {
        return 0;
    }
    
}
