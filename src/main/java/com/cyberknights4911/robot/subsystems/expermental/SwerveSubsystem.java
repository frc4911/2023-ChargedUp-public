package com.cyberknights4911.robot.subsystems.expermental;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.constants.CotsFalconSwerveConstants;
import com.cyberknights4911.robot.constants.Constants.Swerve;
import com.cyberknights4911.robot.constants.Ports;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveModules;
    private final Pigeon2 gyro;
    private final SwerveDriveKinematics kinematics;
    private final CotsFalconSwerveConstants physicalSwerveModule;
    private final CtreConfigs ctreConfigs;

    public SwerveSubsystem() {
        gyro = new Pigeon2(Ports.PIGEON, Constants.CANIVORE_NAME);
        gyro.configFactoryDefault();
        zeroGyro();
        
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Swerve.WHEEL_BASE / 2.0, Swerve.TRACK_WIDTH / 2.0),
            new Translation2d(Swerve.WHEEL_BASE / 2.0, -Swerve.TRACK_WIDTH / 2.0),
            new Translation2d(-Swerve.WHEEL_BASE / 2.0, Swerve.TRACK_WIDTH / 2.0),
            new Translation2d(-Swerve.WHEEL_BASE / 2.0, -Swerve.TRACK_WIDTH / 2.0)
        );

        physicalSwerveModule = CotsFalconSwerveConstants.SDSMK4i(
            CotsFalconSwerveConstants.DriveGearRatios.SDSMK4i_L2
        );

        ctreConfigs = new CtreConfigs(physicalSwerveModule);

        swerveModules = new SwerveModule[] {
            new SwerveModule(
                0,
                Ports.ROBOT_2022_FRONT_LEFT_DRIVE,
                Ports.ROBOT_2022_FRONT_LEFT_STEER,
                Ports.ROBOT_2022_FRONT_LEFT_CANCODER,
                Rotation2d.fromDegrees(150.38),
                physicalSwerveModule,
                ctreConfigs
            ),
            new SwerveModule(
                1,
                Ports.ROBOT_2022_FRONT_RIGHT_DRIVE,
                Ports.ROBOT_2022_FRONT_RIGHT_STEER,
                Ports.ROBOT_2022_FRONT_RIGHT_CANCODER,
                Rotation2d.fromDegrees(2.29),
                physicalSwerveModule,
                ctreConfigs
            ),
            new SwerveModule(
                2,
                Ports.ROBOT_2022_BACK_LEFT_DRIVE,
                Ports.ROBOT_2022_BACK_LEFT_STEER,
                Ports.ROBOT_2022_BACK_LEFT_CANCODER,
                Rotation2d.fromDegrees(83.23),
                physicalSwerveModule,
                ctreConfigs
            ),
            new SwerveModule(
                3,
                Ports.ROBOT_2022_BACK_RIGHT_DRIVE,
                Ports.ROBOT_2022_BACK_RIGHT_STEER,
                Ports.ROBOT_2022_BACK_RIGHT_CANCODER,
                Rotation2d.fromDegrees(244.07),
                physicalSwerveModule,
                ctreConfigs
            )
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());
    }

    public void drive(
            Translation2d translation,
            double rotation,
            boolean fieldRelative,
            boolean isOpenLoop
    ) {
        SwerveModuleState[] swerveModuleStates =
            kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.MAX_SPEED);

        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule swerveModule = swerveModules[i];
            swerveModule.setDesiredState(swerveModuleStates[swerveModule.moduleNumber], isOpenLoop);
        }
    }    

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.MAX_SPEED);
        
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule swerveModule = swerveModules[i];
            swerveModule.setDesiredState(desiredStates[swerveModule.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < states.length; i++) {
            SwerveModule swerveModule = swerveModules[i];
            states[swerveModule.moduleNumber] = swerveModule.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++) {
            SwerveModule swerveModule = swerveModules[i];
            positions[swerveModule.moduleNumber] = swerveModule.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return Swerve.INVERT_GYRO ?
            Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule swerveModule = swerveModules[i];

            SmartDashboard.putNumber(
                new StringBuilder()
                    .append("Mod ")
                    .append(swerveModule.moduleNumber)
                    .append( " Cancoder")
                    .toString(),
                swerveModule.getCanCoder().getDegrees()
            );
            SmartDashboard.putNumber(
                new StringBuilder()
                    .append("Mod ")
                    .append(swerveModule.moduleNumber)
                    .append( " Integrated")
                    .toString(),
                swerveModule.getPosition().angle.getDegrees()
            );
            SmartDashboard.putNumber(
                new StringBuilder()
                    .append("Mod ")
                    .append(swerveModule.moduleNumber)
                    .append( " Velocity")
                    .toString(),
                swerveModule.getState().speedMetersPerSecond
            );
        }
    }
}
