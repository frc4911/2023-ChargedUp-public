package com.cyberknights4911.robot.commands.auto;

import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Factory for creating PathPlanner auto path. */
public final class PathPlannerCommandFactory {
    
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<Pose2d> resetPose;
    private final PIDConstants translationConstants;
    private final PIDConstants rotationConstants;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Subsystem[] driveRequirements;

    /**
     * Creates an auto command factory.
     * 
     * @param poseSupplier A function that supplies the robot pose - use one of the odometry
     *     classes to provide this.
     * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will
     *     typically be called once at the beginning of an auto.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param outputModuleStates A function that takes raw output module states from path
     *     following commands
     * @param driveRequirements The subsystems that the path following commands should require.
     *     Usually just a Drive subsystem.
     */
    public PathPlannerCommandFactory(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        SwerveDriveKinematics kinematics,
        Consumer<SwerveModuleState[]> outputModuleStates,
        Subsystem... driveRequirements
    ) {
        this.poseSupplier = poseSupplier;
        this.resetPose = resetPose;
        this.kinematics = kinematics;
        this.outputModuleStates = outputModuleStates;
        this.driveRequirements = driveRequirements;
        
        // PID constants to correct for translation error (used to create the X and Y PID controllers)
        translationConstants = new PIDConstants(5.0, 0.0, 0.0);
        // PID constants to correct for rotation error (used to create the rotation controller)
        rotationConstants = new PIDConstants(5.0, 0.0, 0.0);
    }

    /**
     * Create a complete autonomous command group. This will reset the robot pose at the begininng
     * of the first path, follow paths, trigger events during path following, and run commands
     * between paths with stop events.
     * 
     * @param pathGroup Path group to follow during the auto
     * @param eventMap Map of event marker names to the commands that should run when reaching that
     *     marker.
     * @return Autonomous PathPlanner command
     */
    public Command createAutoCommand(
        List<PathPlannerTrajectory> pathGroup,
        HashMap<String, Command> eventMap
    ) {
        return new SwerveAutoBuilder(
            poseSupplier,
            resetPose,
            kinematics,
            translationConstants,
            rotationConstants,
            outputModuleStates,
            eventMap,
            driveRequirements
        ).fullAuto(pathGroup);
    }
}
