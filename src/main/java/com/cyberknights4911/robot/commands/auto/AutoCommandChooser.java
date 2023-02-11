package com.cyberknights4911.robot.commands.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.cyberknights4911.robot.commands.MoveHoodCommand;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.hood.HoodSubsystem.HoodPositions;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class AutoCommandChooser {

    private final Subsystems subsystems;
    private final LoggedDashboardChooser<Command> loggedDashboardChooser;
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<Pose2d> resetPose;
    private final PIDConstants translationConstants;
    private final PIDConstants rotationConstants;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<SwerveModuleState[]> outputModuleStates;

    public AutoCommandChooser(Subsystems subsystems) {
        this.subsystems = subsystems;

        loggedDashboardChooser = new LoggedDashboardChooser<Command>("Auto Routine");
        
        // PID constants to correct for translation error (used to create the X and Y PID controllers)

        translationConstants = new PIDConstants(2.0, 1.0, 0.0);
        // PID constants to correct for rotation error (used to create the rotation controller)
        rotationConstants = new PIDConstants(7.0, 0.0, 0.0);


        poseSupplier = subsystems.getSwerveSubsystem()::getPose;
        resetPose = subsystems.getSwerveSubsystem()::setPose;
        kinematics = subsystems.getSwerveSubsystem().getKinematics();
        outputModuleStates = subsystems.getSwerveSubsystem()::setSwerveModuleStates;

        loggedDashboardChooser.addDefaultOption("Test", getTestCommand());
        loggedDashboardChooser.addOption("Angery", getAngeryCommand());
        loggedDashboardChooser.addOption("Left", getLeftCommand());
        loggedDashboardChooser.addOption("Right", getRightCommand());
        loggedDashboardChooser.addOption("PadCenter", getPadCenterCommand());
        loggedDashboardChooser.addOption("PadLeft", getPadLeftCommand());
        loggedDashboardChooser.addOption("PadRight", getPadRightCommand());
        loggedDashboardChooser.addOption("ScoreLeft", getScoreLeftCommand());
        loggedDashboardChooser.addOption("ScoreRight", getScoreRightCommand());
        loggedDashboardChooser.addOption("RotationTest", getRotationTestCommand());
        loggedDashboardChooser.addOption("Translation", getTranslationTestCommand());
    }

    private SwerveAutoBuilder createSwerveAutoBuilder(
        Map<String, Command> eventMap,
        Subsystem... driveRequirements
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
        );
    }
    private Command getAngeryCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Angery", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getLeftCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Left", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getRightCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Right", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getPadCenterCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("PadCenter", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getPadLeftCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("PadLeft", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getPadRightCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("PadRight", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScoreLeftCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreLeft", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScoreRightCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreRight", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getRotationTestCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("RotationTest", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getTranslationTestCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("TranslationTest", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getTestCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put(
            "hood",
            new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.H2)
        );
        eventMap.put(
            "hoodStow",
            new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.STOWED)
        );

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Test", new PathConstraints(1, 3))
        );

        //TODO: We might need to call a stop on this or set something to stop the robot after it runs.
        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    public Command getCurrentCommand() {
        return loggedDashboardChooser.get();
    }
}
