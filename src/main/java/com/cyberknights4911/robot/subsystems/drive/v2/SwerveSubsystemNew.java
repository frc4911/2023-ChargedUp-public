package com.cyberknights4911.robot.subsystems.drive.v2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.littletonrobotics.junction.Logger;

import com.cyberknights4911.robot.commands.TeleopSwerveCommand;
import com.cyberknights4911.robot.constants.Constants.Swerve;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.control.StickAction;
import com.cyberknights4911.robot.subsystems.drive.GyroIO;
import com.cyberknights4911.robot.subsystems.drive.GyroIOInputsAutoLogged;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystem;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystemNew extends SubsystemBase implements SwerveSubsystem {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule[] swerveModules;
    private final GyroIO gyro;
    private final SwerveDriveKinematics kinematics;
    
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public SwerveSubsystemNew(
        GyroIO gyro,
        SwerveIO frontLeftSwerveIO,
        SwerveIO frontRightSwerveIO,
        SwerveIO backLeftSwerveIO,
        SwerveIO backRightSwerveIO
    ) {
        this.gyro = gyro;
        zeroGyro();
        
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Swerve.WHEEL_BASE / 2.0, Swerve.TRACK_WIDTH / 2.0),
            new Translation2d(Swerve.WHEEL_BASE / 2.0, -Swerve.TRACK_WIDTH / 2.0),
            new Translation2d(-Swerve.WHEEL_BASE / 2.0, Swerve.TRACK_WIDTH / 2.0),
            new Translation2d(-Swerve.WHEEL_BASE / 2.0, -Swerve.TRACK_WIDTH / 2.0)
        );

        swerveModules = new SwerveModule[] {
            new SwerveModule(
                frontLeftSwerveIO,
                SwerveModuleConstants.FRONT_LEFT
            ),
            new SwerveModule(
                frontRightSwerveIO,
                SwerveModuleConstants.FRONT_RIGHT
            ),
            new SwerveModule(
                backLeftSwerveIO,
                SwerveModuleConstants.BACK_LEFT
            ),
            new SwerveModule(
                backRightSwerveIO,
                SwerveModuleConstants.BACK_RIGHT
            )
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        poseEstimator =  new SwerveDrivePoseEstimator(
            kinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01)); // Global measurement standard deviations. X, Y, and theta.
        
        new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());
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

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }    

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /** Used by SwerveControllerCommand in Auto */
    @Override
    public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.MAX_SPEED);
        
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }    

    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.getModuleNumber()] = mod.getPosition();
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

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        poseEstimator.update(getYaw(), getModulePositions());  

        gyro.updateInputs(inputs);
        Logger.getInstance().processInputs("Gyro", inputs);

        for (SwerveModule mod : swerveModules) {
            mod.handlePeriodic();
        }
    }

    @Override
    public Command createDefaultCommand(ControllerBinding controllerBinding) {
        return new TeleopSwerveCommand(
            this,
            controllerBinding.supplierFor(StickAction.FORWARD),
            controllerBinding.supplierFor(StickAction.STRAFE),
            controllerBinding.supplierFor(StickAction.ROTATE),
            () -> false
        );
    }

    @Override
    public void initForPathFollowing() {}
}
