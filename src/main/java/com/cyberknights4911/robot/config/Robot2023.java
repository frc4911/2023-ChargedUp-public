package com.cyberknights4911.robot.config;

import static com.cyberknights4911.robot.constants.Constants.MK4_L2I_DRIVE_REDUCTION;
import static com.cyberknights4911.robot.constants.Constants.MK4_L2I_STEER_REDUCTION;
import static com.cyberknights4911.robot.constants.Constants.MK4_L2I_WHEEL_DIAMETER;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.util.Units;

public class Robot2023 implements RobotConfiguration {

    private static final double FRONT_LEFT_CANCODER_OFFSET_DEGREES = 360.0 - 20.65;
    private static final double FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 360.0 - 159.50;
    private static final double BACK_LEFT_CANCODER_OFFSET_DEGREES = 360.0 - 194.00;
    private static final double BACK_RIGHT_CANCODER_OFFSET_DEGREES = 360.0 - 104.67;

    private static final double STEER_MOTOR_KP = 0.25;
    private static final double STEER_MOTOR_KI = 0.0;
    private static final double STEER_MOTOR_KD = 0.0;
    private static final double STEER_MOTOR_KF = 0.0;

    @Override
    public SwerveConfiguration getSwerveConfiguration() {
        return new SwerveConfiguration.Builder()
                .setWheelbaseLengthInMeters(Constants.ROBOT_WHEEL_LENGTH_METERS)
                .setWheelbaseWidthInMeters(Constants.ROBOT_WHEEL_BASE_METERS)
                .setMaxSpeedInMetersPerSecond(Units.feetToMeters(1))
                .setMaxSpeedInRadiansPerSecondLimit(Math.toRadians(30))
                .setSwerveHeadingKp(0.01)
                .setSwerveHeadingKi(0)
                .setSwerveHeadingKd(0)
                .setSwerveHeadingKf(0)
                .build();
    }

    @Override
    public SwerveModuleConfiguration getFrontRightModuleConstants() {
        return new SwerveModuleConfiguration.Builder()
                .setName("Front Right")
                .setDriveMotorTalonId(Ports.Drive.FRONT_RIGHT_DRIVE)
                .setSteerMotorTalonId(Ports.Drive.FRONT_RIGHT_STEER)
                .setCANCoderId(Ports.Drive.FRONT_RIGHT_CANCODER)
                .setCANCoderOffsetDegrees(FRONT_RIGHT_CANCODER_OFFSET_DEGREES)
                .setWheelDiameter(MK4_L2I_WHEEL_DIAMETER)
                .setDriveReduction(MK4_L2I_DRIVE_REDUCTION)
                .setSteerReduction(MK4_L2I_STEER_REDUCTION)
                .setInvertDrive(true)
                .setInvertSteerMotor(true)
                .setSteerMotorSlot0Kp(STEER_MOTOR_KP)
                .setSteerMotorSlot0Ki(STEER_MOTOR_KI)
                .setSteerMotorSlot0Kd(STEER_MOTOR_KD)
                .setSteerMotorSlot0Kf(STEER_MOTOR_KF)
                .build();
    }

    @Override
    public SwerveModuleConfiguration getFrontLeftModuleConstants() {
        return new SwerveModuleConfiguration.Builder()
                .setName("Front Left")
                .setDriveMotorTalonId(Ports.Drive.FRONT_LEFT_DRIVE)
                .setSteerMotorTalonId(Ports.Drive.FRONT_LEFT_STEER)
                .setCANCoderId(Ports.Drive.FRONT_LEFT_CANCODER)
                .setCANCoderOffsetDegrees(FRONT_LEFT_CANCODER_OFFSET_DEGREES)
                .setWheelDiameter(MK4_L2I_WHEEL_DIAMETER)
                .setDriveReduction(MK4_L2I_DRIVE_REDUCTION)
                .setSteerReduction(MK4_L2I_STEER_REDUCTION)
                .setInvertDrive(true)
                .setInvertSteerMotor(true)
                .setSteerMotorSlot0Kp(STEER_MOTOR_KP)
                .setSteerMotorSlot0Ki(STEER_MOTOR_KI)
                .setSteerMotorSlot0Kd(STEER_MOTOR_KD)
                .setSteerMotorSlot0Kf(STEER_MOTOR_KF)
                .build();
    }

    @Override
    public SwerveModuleConfiguration getBackLeftModuleConstants() {
        return new SwerveModuleConfiguration.Builder()
                .setName("Back Left")
                .setDriveMotorTalonId(Ports.Drive.BACK_LEFT_DRIVE)
                .setSteerMotorTalonId(Ports.Drive.BACK_LEFT_STEER)
                .setCANCoderId(Ports.Drive.BACK_LEFT_CANCODER)
                .setCANCoderOffsetDegrees(BACK_LEFT_CANCODER_OFFSET_DEGREES)
                .setWheelDiameter(MK4_L2I_WHEEL_DIAMETER)
                .setDriveReduction(MK4_L2I_DRIVE_REDUCTION)
                .setSteerReduction(MK4_L2I_STEER_REDUCTION)
                .setInvertDrive(true)
                .setInvertSteerMotor(true)
                .setSteerMotorSlot0Kp(STEER_MOTOR_KP)
                .setSteerMotorSlot0Ki(STEER_MOTOR_KI)
                .setSteerMotorSlot0Kd(STEER_MOTOR_KD)
                .setSteerMotorSlot0Kf(STEER_MOTOR_KF)
                .build();
    }

    @Override
    public SwerveModuleConfiguration getBackRightModuleConstants() {
        return new SwerveModuleConfiguration.Builder()
                .setName("Back Right")
                .setDriveMotorTalonId(Ports.Drive.BACK_RIGHT_DRIVE)
                .setSteerMotorTalonId(Ports.Drive.BACK_RIGHT_STEER)
                .setCANCoderId(Ports.Drive.BACK_RIGHT_CANCODER)
                .setCANCoderOffsetDegrees(BACK_RIGHT_CANCODER_OFFSET_DEGREES)
                .setWheelDiameter(MK4_L2I_WHEEL_DIAMETER)
                .setDriveReduction(MK4_L2I_DRIVE_REDUCTION)
                .setSteerReduction(MK4_L2I_STEER_REDUCTION)
                .setInvertDrive(true)
                .setInvertSteerMotor(true)
                .setSteerMotorSlot0Kp(STEER_MOTOR_KP)
                .setSteerMotorSlot0Ki(STEER_MOTOR_KI)
                .setSteerMotorSlot0Kd(STEER_MOTOR_KD)
                .setSteerMotorSlot0Kf(STEER_MOTOR_KF)
                .build();
    }
}
