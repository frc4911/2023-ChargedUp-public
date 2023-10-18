package com.cyberknights4911.robot.model.quickdrop.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.cyberknights4911.robot.model.quickdrop.QuickDropConstants;
import com.cyberknights4911.robot.model.quickdrop.QuickDropPorts;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class ShooterIOReal implements ShooterIO {
    private static final double HOOD_RANGE_UNITS = 20387.0;

    private final WPI_TalonFX hoodMotor;
    private final WPI_TalonFX flywheelRightMotor;
    private final WPI_TalonFX flywheelLeftMotor;
    private final CtreError ctreError;
    private final double hoodStartPosition;

    public ShooterIOReal(TalonFXFactory rioFXFactory, TalonFXFactory canivoreFXFactory, CtreError ctreError) {
        this.ctreError = ctreError;
        hoodMotor = canivoreFXFactory.createTalon(QuickDropPorts.Shooter.HOOD_MOTOR);
        flywheelRightMotor = rioFXFactory.createTalon(QuickDropPorts.Shooter.FLYWHEEL_RIGHT_MOTOR);
        flywheelLeftMotor = rioFXFactory.createTalon(QuickDropPorts.Shooter.FLYWHEEL_LEFT_MOTOR);
        configMotors();
        hoodStartPosition = hoodMotor.getSelectedSensorPosition();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.appliedVoltsHood = hoodMotor.getMotorOutputVoltage();
        inputs.appliedVoltsLeft = flywheelLeftMotor.getMotorOutputVoltage();
        inputs.appliedVoltsRight = flywheelRightMotor.getMotorOutputVoltage();
        inputs.velocityRpm = flywheelRightMotor.getSelectedSensorVelocity() * 10 / 2048;
        inputs.hoodPosition = hoodMotor.getSelectedSensorPosition();
    }

    @Override
    public void setShooterSpeed(double speed) {
        flywheelRightMotor.set(ControlMode.Velocity, speed);
    }

    @Override
    public void setHoodPosition(double percent) {
        double desiredPosition = hoodStartPosition + HOOD_RANGE_UNITS * percent;
        hoodMotor.set(ControlMode.Position, desiredPosition);
    }

    private void configMotors() {
        hoodMotor.setInverted(true);

        ctreError.checkError(flywheelRightMotor.configStatorCurrentLimit(
            QuickDropConstants.Shooter.FLYWHEEL_RIGHT_STATOR_LIMIT, ctreError.canTimeoutMs()));

        ctreError.checkError(flywheelRightMotor.config_kP(
            0, QuickDropConstants.Shooter.FLYWHEEL_P, ctreError.canTimeoutMs()));
        ctreError.checkError(flywheelRightMotor.config_kI(
            0, QuickDropConstants.Shooter.FLYWHEEL_I, ctreError.canTimeoutMs()));
        ctreError.checkError(flywheelRightMotor.config_kD(
            0, QuickDropConstants.Shooter.FLYWHEEL_D, ctreError.canTimeoutMs()));
        ctreError.checkError(flywheelRightMotor.config_kF(
            0, QuickDropConstants.Shooter.FLYWHEEL_F, ctreError.canTimeoutMs()));
        ctreError.checkError(flywheelRightMotor.config_IntegralZone(
            0, QuickDropConstants.Shooter.FLYWHEEL_INTEGRAL_ZONE, ctreError.canTimeoutMs()));
        ctreError.checkError(flywheelRightMotor.configClosedloopRamp(
            QuickDropConstants.Shooter.FLYWHEEL_CLOSED_RAMP, ctreError.canTimeoutMs()));
        ctreError.checkError(flywheelRightMotor.configAllowableClosedloopError(
            0, QuickDropConstants.Shooter.FLYWHEEL_CLOSED_RAMP, ctreError.canTimeoutMs()));

        flywheelLeftMotor.configFactoryDefault();
        flywheelLeftMotor.follow(flywheelRightMotor);
        flywheelLeftMotor.setInverted(true);
        ctreError.checkError(flywheelLeftMotor.configStatorCurrentLimit(
            QuickDropConstants.Shooter.FLYWHEEL_LEFT_STATOR_LIMIT, ctreError.canTimeoutMs()));

        hoodMotor.configFactoryDefault();
        ctreError.checkError(hoodMotor.setSelectedSensorPosition(
            0, 0, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.config_kP(
            0, QuickDropConstants.Shooter.HOOD_I, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.config_kI(
            0, QuickDropConstants.Shooter.HOOD_I, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.config_kD(
            0, QuickDropConstants.Shooter.HOOD_D, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.config_kF(
            0, QuickDropConstants.Shooter.HOOD_F, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.config_IntegralZone(
            0, QuickDropConstants.Shooter.HOOD_INTEGRAL_ZONE, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.configClosedloopRamp(
            QuickDropConstants.Shooter.HOOD_CLOSED_RAMP, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.configAllowableClosedloopError(
            0, QuickDropConstants.Shooter.HOOD_CLOSED_ERROR, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.configMotionCruiseVelocity(
            QuickDropConstants.Shooter.HOOD_MOTION_CRUISE_VELOCITY, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.configMotionAcceleration(
            QuickDropConstants.Shooter.HOOD_MOTION_ACCELERATION, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.configMotionSCurveStrength(
            QuickDropConstants.Shooter.HOOD_MOTION_S_CURVE_STRENGTH, ctreError.canTimeoutMs()));
        ctreError.checkError(hoodMotor.configStatorCurrentLimit(
            QuickDropConstants.Shooter.HOOD_STATOR_LIMIT, ctreError.canTimeoutMs()));
    }
}