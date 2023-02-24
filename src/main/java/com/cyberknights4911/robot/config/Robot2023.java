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
        return new SwerveConfiguration(
                Constants.ROBOT_LENGTH_METERS,
                Constants.ROBOT_WIDTH_METERS,
                Units.feetToMeters(1), //7.5 Max speed in feet per second in auto: Theoretical max is 17.065 ft per second
                Math.toRadians(30), // 150 Max change in degrees per second
                0.01, 0, 0, 0 // kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf
        );
    }

    @Override
    public SwerveModuleConfiguration getFrontRightModuleConstants() {
        SwerveModuleConfiguration frontRightModuleConstants = new SwerveModuleConfiguration();

        frontRightModuleConstants.kName = "Front Right";
        frontRightModuleConstants.kDriveMotorTalonId = Ports.Drive.FRONT_RIGHT_DRIVE;
        frontRightModuleConstants.kSteerMotorTalonId = Ports.Drive.FRONT_RIGHT_STEER;
        frontRightModuleConstants.kCANCoderId = Ports.Drive.FRONT_RIGHT_CANCODER;
        frontRightModuleConstants.kCANCoderOffsetDegrees = FRONT_RIGHT_CANCODER_OFFSET_DEGREES;
        frontRightModuleConstants.kWheelDiameter = MK4_L2I_WHEEL_DIAMETER;
        frontRightModuleConstants.kDriveReduction = MK4_L2I_DRIVE_REDUCTION;
        frontRightModuleConstants.kSteerReduction = MK4_L2I_STEER_REDUCTION;
        frontRightModuleConstants.kInvertDrive = true;
        frontRightModuleConstants.kInvertSteerMotor = true;
        frontRightModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        frontRightModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        frontRightModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        frontRightModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return frontRightModuleConstants;
    }

    @Override
    public SwerveModuleConfiguration getFrontLeftModuleConstants() {
        SwerveModuleConfiguration frontLeftModuleConstants = new SwerveModuleConfiguration();

        frontLeftModuleConstants.kName = "Front Left";
        frontLeftModuleConstants.kDriveMotorTalonId = Ports.Drive.FRONT_LEFT_DRIVE;
        frontLeftModuleConstants.kSteerMotorTalonId = Ports.Drive.FRONT_LEFT_STEER;
        frontLeftModuleConstants.kCANCoderId = Ports.Drive.FRONT_LEFT_CANCODER;
        frontLeftModuleConstants.kCANCoderOffsetDegrees = FRONT_LEFT_CANCODER_OFFSET_DEGREES;
        frontLeftModuleConstants.kWheelDiameter = MK4_L2I_WHEEL_DIAMETER;
        frontLeftModuleConstants.kDriveReduction = MK4_L2I_DRIVE_REDUCTION;
        frontLeftModuleConstants.kSteerReduction = MK4_L2I_STEER_REDUCTION;
        frontLeftModuleConstants.kInvertDrive = true;
        frontLeftModuleConstants.kInvertSteerMotor = true;
        frontLeftModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        frontLeftModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        frontLeftModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        frontLeftModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return frontLeftModuleConstants;
    }

    @Override
    public SwerveModuleConfiguration getBackLeftModuleConstants() {
        SwerveModuleConfiguration backLeftModuleConstants = new SwerveModuleConfiguration();

        backLeftModuleConstants.kName = "Back Left";
        backLeftModuleConstants.kDriveMotorTalonId = Ports.Drive.BACK_LEFT_DRIVE;
        backLeftModuleConstants.kSteerMotorTalonId = Ports.Drive.BACK_LEFT_STEER;
        backLeftModuleConstants.kCANCoderId = Ports.Drive.BACK_LEFT_CANCODER;
        backLeftModuleConstants.kCANCoderOffsetDegrees = BACK_LEFT_CANCODER_OFFSET_DEGREES;
        backLeftModuleConstants.kWheelDiameter = MK4_L2I_WHEEL_DIAMETER;
        backLeftModuleConstants.kDriveReduction = MK4_L2I_DRIVE_REDUCTION;
        backLeftModuleConstants.kSteerReduction = MK4_L2I_STEER_REDUCTION;
        backLeftModuleConstants.kInvertDrive = true;
        backLeftModuleConstants.kInvertSteerMotor = true;
        backLeftModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        backLeftModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        backLeftModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        backLeftModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return backLeftModuleConstants;
    }

    @Override
    public SwerveModuleConfiguration getBackRightModuleConstants() {
        SwerveModuleConfiguration backRightModuleConstants = new SwerveModuleConfiguration();

        backRightModuleConstants.kName = "Back Right";
        backRightModuleConstants.kDriveMotorTalonId = Ports.Drive.BACK_RIGHT_DRIVE;
        backRightModuleConstants.kSteerMotorTalonId = Ports.Drive.BACK_RIGHT_STEER;
        backRightModuleConstants.kCANCoderId = Ports.Drive.BACK_RIGHT_CANCODER;
        backRightModuleConstants.kCANCoderOffsetDegrees = BACK_RIGHT_CANCODER_OFFSET_DEGREES;
        backRightModuleConstants.kWheelDiameter = MK4_L2I_WHEEL_DIAMETER;
        backRightModuleConstants.kDriveReduction = MK4_L2I_DRIVE_REDUCTION;
        backRightModuleConstants.kSteerReduction = MK4_L2I_STEER_REDUCTION;
        backRightModuleConstants.kInvertDrive = true;
        backRightModuleConstants.kInvertSteerMotor = true;
        backRightModuleConstants.kSteerMotorSlot0Kp = STEER_MOTOR_KP;
        backRightModuleConstants.kSteerMotorSlot0Ki = STEER_MOTOR_KI;
        backRightModuleConstants.kSteerMotorSlot0Kd = STEER_MOTOR_KD;
        backRightModuleConstants.kSteerMotorSlot0Kf = STEER_MOTOR_KF;

        return backRightModuleConstants;
    }
}
