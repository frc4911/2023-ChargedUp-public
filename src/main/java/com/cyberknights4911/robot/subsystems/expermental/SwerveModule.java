package com.cyberknights4911.robot.subsystems.expermental;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.constants.Constants.Swerve;
import com.cyberknights4911.robot.util.Conversions;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANCoder angleEncoder;
    
    private final CotsFalconSwerveConstants physicalSwerveModule;
    private final CtreConfigs ctreConfigs;

    private final SimpleMotorFeedforward feedforward;

    public SwerveModule(
            int moduleNumber,
            int driveMotorId,
            int angleMotorId,
            int canCoderId,
            Rotation2d angleOffset,
            CotsFalconSwerveConstants physicalSwerveModule,
            CtreConfigs ctreConfigs
    ) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;
        this.physicalSwerveModule = physicalSwerveModule;
        this.ctreConfigs = ctreConfigs;

        feedforward = new SimpleMotorFeedforward(
            Swerve.DRIVE_KS,
            Swerve.DRIVE_KV,
            Swerve.DRIVE_KA
        );
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(canCoderId, Constants.CANIVORE_NAME);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(angleMotorId, Constants.CANIVORE_NAME);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorId, Constants.CANIVORE_NAME);
        configDriveMotor();

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
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(
                desiredState.speedMetersPerSecond,
                physicalSwerveModule.wheelCircumference,
                physicalSwerveModule.driveGearRatio
            );
            driveMotor.set(
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
        
        angleMotor.set(
            ControlMode.Position,
            Conversions.degreesToFalcon(angle.getDegrees(), physicalSwerveModule.angleGearRatio)
        );
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
            Conversions.falconToDegrees(
                angleMotor.getSelectedSensorPosition(),
                physicalSwerveModule.angleGearRatio
            )
        );
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
            getCanCoder().getDegrees() - angleOffset.getDegrees(),
            physicalSwerveModule.angleGearRatio
        );
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(physicalSwerveModule.angleMotorInvert);
        angleMotor.setNeutralMode(Swerve.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(physicalSwerveModule.driveMotorInvert);
        driveMotor.setNeutralMode(Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(
                driveMotor.getSelectedSensorVelocity(),
                physicalSwerveModule.wheelCircumference,
                physicalSwerveModule.driveGearRatio
            ), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(
                driveMotor.getSelectedSensorPosition(),
                physicalSwerveModule.wheelCircumference,
                physicalSwerveModule.driveGearRatio
            ), 
            getAngle()
        );
    }
}
