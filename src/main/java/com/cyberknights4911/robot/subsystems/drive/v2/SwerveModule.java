package com.cyberknights4911.robot.subsystems.drive.v2;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.constants.Constants.Swerve;
import com.cyberknights4911.robot.util.Conversions;

public class SwerveModule {
    private Rotation2d lastAngle;

    private final String moduleName;
    private final int moduleNumber;
    private final SwerveIO swerveIO;
    private final CotsFalconSwerveConstants physicalSwerveModule;
    private final SimpleMotorFeedforward feedforward;
    
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    public SwerveModule(
        int moduleNumber,
        SwerveIO swerveIO,
        CotsFalconSwerveConstants physicalSwerveModule
    ) {
        this.swerveIO = swerveIO;
        this.moduleNumber = moduleNumber;
        this.physicalSwerveModule = physicalSwerveModule;
        this.moduleName = String.format("Swerve %d", moduleNumber);

        feedforward = new SimpleMotorFeedforward(
            Swerve.DRIVE_KS,
            Swerve.DRIVE_KV,
            Swerve.DRIVE_KA
        );

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous
         * controller which CTRE and Rev onboard is not */
        desiredState = CtreModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Swerve.MAX_SPEED;
            swerveIO.setDrive(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(
                desiredState.speedMetersPerSecond,
                physicalSwerveModule.wheelCircumference,
                physicalSwerveModule.driveGearRatio
            );
            swerveIO.setDrive(
                ControlMode.Velocity,
                velocity,
                DemandType.ArbitraryFeedForward,
                feedforward.calculate(desiredState.speedMetersPerSecond)
            );
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        boolean isSpeedLessThanOnePercent =
            Math.abs(desiredState.speedMetersPerSecond) <= Swerve.MAX_SPEED * 0.01;
        Rotation2d angle = isSpeedLessThanOnePercent ? lastAngle : desiredState.angle;
        
        swerveIO.setAngle(
            ControlMode.Position,
            Conversions.degreesToFalcon(angle.getDegrees(), physicalSwerveModule.angleGearRatio)
        );
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
            Conversions.falconToDegrees(
                swerveIO.getAngleSensorPosition(),
                physicalSwerveModule.angleGearRatio
            )
        );
    }

    public double getAngleEncoderDegrees() {
        return swerveIO.getAngleEncoderDegrees();
    }

    public void resetToAbsolute() {
        swerveIO.resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(
                swerveIO.getDriveSensorVelocity(),
                physicalSwerveModule.wheelCircumference,
                physicalSwerveModule.driveGearRatio
            ), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(
                swerveIO.getDriveSensorPosition(),
                physicalSwerveModule.wheelCircumference,
                physicalSwerveModule.driveGearRatio
            ), 
            getAngle()
        );
    }

    public void handlePeriodic() {
        swerveIO.updateInputs(inputs);
        Logger.getInstance().processInputs(moduleName, inputs);
    }

    public int getModuleNumber() {
        return moduleNumber;
    }
}
