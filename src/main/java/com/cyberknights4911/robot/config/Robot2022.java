package com.cyberknights4911.robot.config;

import static com.cyberknights4911.robot.constants.Constants.MK4_L2I_DRIVE_REDUCTION;
import static com.cyberknights4911.robot.constants.Constants.MK4_L2I_STEER_REDUCTION;
import static com.cyberknights4911.robot.constants.Constants.MK4_L2I_WHEEL_DIAMETER;

import com.cyberknights4911.robot.constants.Ports;
import com.cyberknights4911.robot.sensors.IMU.ImuType;

import edu.wpi.first.math.util.Units;

public class Robot2022 implements RobotConfiguration {
    
    private static final double FRONT_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET = 883.0;
    private static final double FRONT_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET = 1683.0;
    private static final double BACK_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET = 3451.0;
    private static final double BACK_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET = -327.0;

    private static final int FRONT_RIGHT_CANCODER_OFFSET_DEGREES = 358;
    private static final int FRONT_LEFT_CANCODER_OFFSET_DEGREES = 210;
    private static final int BACK_LEFT_CANCODER_OFFSET_DEGREES = 278;
    private static final int BACK_RIGHT_CANCODER_OFFSET_DEGREES = 116;

    private static final double STEER_MOTOR_KP = 0.25;
    private static final double STEER_MOTOR_KI = 0.0;
    private static final double STEER_MOTOR_KD = 0.0;
    private static final double STEER_MOTOR_KF = 0.0;

    @Override
    public SwerveConfiguration getSwerveConfiguration() {
        return new SwerveConfiguration(
                Units.inchesToMeters(23.75),
                Units.inchesToMeters(20.75),
                Units.feetToMeters(7.5), //7.5 Max speed in feet per second in auto: Theoretical max is 17.065 ft per second
                Math.toRadians(100), // 150 Max change in degrees per second
                0.01, 0, 0, 0 // kSwerveHeadingKp,kSwerveHeadingKi,kSwerveHeadingKp,kSwerveHeadingKf
        );
    }

    @Override
    public SwerveModuleConfiguration getFrontRightModuleConstants() {
        SwerveModuleConfiguration frontRightModuleConstants = new SwerveModuleConfiguration();

        frontRightModuleConstants.kName = "Front Right";
        frontRightModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_FRONT_RIGHT_DRIVE;
        frontRightModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_FRONT_RIGHT_STEER;
        frontRightModuleConstants.kSteerMotorEncoderHomeOffset = FRONT_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        frontRightModuleConstants.kCANCoderId = Ports.ROBOT_2022_FRONT_RIGHT_CANCODER;
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
        frontLeftModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_FRONT_LEFT_DRIVE;
        frontLeftModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_FRONT_LEFT_STEER;
        frontLeftModuleConstants.kSteerMotorEncoderHomeOffset = FRONT_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        frontLeftModuleConstants.kCANCoderId = Ports.ROBOT_2022_FRONT_LEFT_CANCODER;
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
        backLeftModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_BACK_LEFT_DRIVE;
        backLeftModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_BACK_LEFT_STEER;
        backLeftModuleConstants.kSteerMotorEncoderHomeOffset = BACK_LEFT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        backLeftModuleConstants.kCANCoderId = Ports.ROBOT_2022_BACK_LEFT_CANCODER;
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
        backRightModuleConstants.kDriveMotorTalonId = Ports.ROBOT_2022_BACK_RIGHT_DRIVE;
        backRightModuleConstants.kSteerMotorTalonId = Ports.ROBOT_2022_BACK_RIGHT_STEER;
        backRightModuleConstants.kSteerMotorEncoderHomeOffset = BACK_RIGHT_STEER_MOTOR_ENCODER_HOME_OFFSET;
        backRightModuleConstants.kCANCoderId = Ports.ROBOT_2022_BACK_RIGHT_CANCODER;
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

    @Override
    public ImuType getImuType() {
        return ImuType.PIGEON2;
    }

    // @Override
    // public LimelightConfiguration getLimelightConfiguration()
    // {
    //     return new LimelightConfiguration(
    //             1, // label id
    //             LimelightConfiguration.Type.Shooter,
    //             "Shooter Limelight #1", // name
    //             "limelight-shoot", // table name
    //             Units.inchesToMeters(40), // height
    //             // Distance between center of shooter and limelight's camera lens
    //             new Pose2d(0.0, Units.inchesToMeters(-7.0), Rotation2d.identity()), // shooter to lens
    //             Rotation2d.fromDegrees(50.00), // horizontalPlaneToLens,
    //             65.0, //64.03840065743408,
    //             50.0 //50.34836606499798
    //     );
    // }
}
