package com.cyberknights4911.robot.drive.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import com.cyberknights4911.robot.control.DriveStickAction;
import com.cyberknights4911.robot.control.StickBinding;
import com.cyberknights4911.robot.model.wham.drive.AutoBalanceCommand;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule[] swerveModules;
    private final GyroIO gyro;
    private final SwerveDriveConstants swerveDriveConstants;
    private final SwerveDriveKinematics kinematics;
    
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public SwerveSubsystem(SwerveSubsystemArgs args) {
        this.gyro = args.gyroIO();
        this.swerveDriveConstants = args.swerveDriveConstants();
        zeroGyro();

        kinematics = new SwerveDriveKinematics(
            new Translation2d(swerveDriveConstants.wheelBase() / 2.0, swerveDriveConstants.trackWidth() / 2.0),
            new Translation2d(swerveDriveConstants.wheelBase() / 2.0, -swerveDriveConstants.trackWidth() / 2.0),
            new Translation2d(-swerveDriveConstants.wheelBase() / 2.0, swerveDriveConstants.trackWidth() / 2.0),
            new Translation2d(-swerveDriveConstants.wheelBase() / 2.0, -swerveDriveConstants.trackWidth() / 2.0)
        );

        swerveModules = new SwerveModule[] {
            args.frontLeftSwerveModule(),
            args.frontRightSwerveModule(),
            args.backLeftSwerveModule(),
            args.backRightSwerveModule()
        };

        for (SwerveModule swerveModule : swerveModules) {
            if (swerveModule == null) {
                throw new IllegalArgumentException("Four modules required, numbered 0 to 3");
            }
        }

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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, swerveDriveConstants.maxSpeed());

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber() - 1], isOpenLoop);
        }
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /** Used by SwerveControllerCommand in Auto */
    public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, swerveDriveConstants.maxSpeed());
        
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber() - 1], false);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public CommandBase createAutobalanceCommand() {
        CommandBase balanceCommand =  new AutoBalanceCommand() {

            @Override
            public void driveTeleop(Translation2d translation, double rotation, boolean fieldRelative) {
                drive(translation, rotation, fieldRelative, true);
            }

            @Override
            public Translation2d getTilt() {
                double roll = swerveDriveConstants.invertGyro()
                    ? 360 - gyro.getRoll()
                    : gyro.getRoll();
                double pitch = swerveDriveConstants.invertGyro()
                    ? 360 - gyro.getPitch()
                    : gyro.getPitch();
                
                Translation2d tilt = new Translation2d(roll, pitch); 
                return tilt.times(swerveDriveConstants.maxSpeed());
            }
        };
        balanceCommand.addRequirements(this);
        return balanceCommand;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.getModuleNumber() - 1] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.getModuleNumber() - 1] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double yaw = gyro.getYaw();

        return swerveDriveConstants.invertGyro()
            ? Rotation2d.fromDegrees(360 - yaw)
            : Rotation2d.fromDegrees(yaw);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        poseEstimator.update(getYaw(), getModulePositions());  

        gyro.updateInputs(inputs);
        Logger.getInstance().processInputs("Gyro", inputs);

        for (SwerveModule mod : swerveModules) {
            mod.handlePeriodic();
        }
    }

    public Command createTeleopDriveCommand(StickBinding stickBinding) {
        DoubleSupplier translationSupplier = stickBinding.supplierFor(DriveStickAction.FORWARD);
        DoubleSupplier strafeSupplier = stickBinding.supplierFor(DriveStickAction.STRAFE);
        DoubleSupplier rotationSupplier = stickBinding.supplierFor(DriveStickAction.ROTATE);

        return Commands.run(
            () -> {
                double translationValue = MathUtil.applyDeadband(translationSupplier.getAsDouble(),
                    swerveDriveConstants.stickDeadband());
                double strafeValue = MathUtil.applyDeadband(strafeSupplier.getAsDouble(),
                    swerveDriveConstants.stickDeadband());
                double rotationValue = MathUtil.applyDeadband(rotationSupplier.getAsDouble(),
                    swerveDriveConstants.stickDeadband());

                drive(
                    new Translation2d(translationValue, strafeValue).times(swerveDriveConstants.maxSpeed()),
                    rotationValue * swerveDriveConstants.maxAngularVelocity(),
                    /* fieldRelative = */ false,
                    /* isOpenLoop = */ true
                );
            },
            /* requirements = */ this);
    }
}
