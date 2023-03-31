package com.cyberknights4911.robot.commands.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.cyberknights4911.robot.commands.AutoBalanceCommand;
import com.cyberknights4911.robot.commands.MoveArmMotionMagicCommand;
import com.cyberknights4911.robot.commands.SlurppCommand;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystemCurrent;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
        translationConstants = new PIDConstants(4.3, 0, 0.0);

        // PID constants to correct for rotation error (used to create the rotation controller)
        rotationConstants = new PIDConstants(2.0, 0.0, 0.0);

        poseSupplier = subsystems.getSwerveSubsystem()::getPose;
        resetPose = subsystems.getSwerveSubsystem()::setPose;
        kinematics = subsystems.getSwerveSubsystem().getKinematics();
        outputModuleStates = subsystems.getSwerveSubsystem()::setSwerveModuleStates;

        //loggedDashboardChooser.addDefaultOption("Score", getScoreCommand());

        loggedDashboardChooser.addDefaultOption("AutoBalance", getAutoBalanceCommand());
        loggedDashboardChooser.addOption("Balance Score Leave", getBalanceLeaveCommand());

        loggedDashboardChooser.addOption("Score And Leave", getScoreAndLeaveCommand());
        loggedDashboardChooser.addOption("2 Substation", getScore2SubstationCommand());
        loggedDashboardChooser.addOption("2 Guardrail", getScore2GuardrailCommand());
        loggedDashboardChooser.addOption("3 Substation", getScore3SubstationCollect());
        loggedDashboardChooser.addOption("3 Guardrail", getScore3GuardrailCommand());

        loggedDashboardChooser.addOption("RotationTest", getRotationTestCommand());
        loggedDashboardChooser.addOption("TranslationTest", getTranslationTestCommand());
        loggedDashboardChooser.addOption("Test", getTestCommand());

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

    private Command getAutoBalanceCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("coneScoreSlow", Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(-0.40), subsystems.getSlurppSubsystem()));
        eventMap.put("stopSlurrp",Commands.runOnce(() -> subsystems.getSlurppSubsystem().stop(), subsystems.getSlurppSubsystem()));
        eventMap.put("autoBalance", new AutoBalanceCommand((SwerveSubsystemCurrent) subsystems.getSwerveSubsystem()));


        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("AutoBalance", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScoreAndLeaveCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("moveL3", MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L3));//Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(-0.40), subsystems.getSlurppSubsystem()));
        eventMap.put("coneScoreSlow", Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(-0.40), subsystems.getSlurppSubsystem()));
        eventMap.put("stopSlurrp",Commands.runOnce(() -> subsystems.getSlurppSubsystem().stop(), subsystems.getSlurppSubsystem()));


        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreAndLeave", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    // private Command getScoreCommand() {
    //     HashMap<String, Command> eventMap = new HashMap<>();

    //     Command autoCommand = createSwerveAutoBuilder(
    //         eventMap,
    //         subsystems.getSwerveSubsystem()
    //     ).fullAuto(
    //         PathPlanner.loadPathGroup("score", new PathConstraints(1, 3))
    //     );

    //     return new InstantCommand(
    //         () -> subsystems.getSwerveSubsystem().initForPathFollowing()
    //     ).andThen(autoCommand);
    // }

    private Command getBalanceLeaveCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("coneScoreSlow", Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(-0.4), subsystems.getSlurppSubsystem()));
        eventMap.put("stopSlurpp",Commands.runOnce(() -> subsystems.getSlurppSubsystem().stop(), subsystems.getSlurppSubsystem()));
        eventMap.put("autoBalance", new AutoBalanceCommand((SwerveSubsystemCurrent) subsystems.getSwerveSubsystem()));

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Balance and Leave", new PathConstraints(0.5, 0.5))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }


    private Command getScore2SubstationCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("coneCollect", Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(0.4), subsystems.getSlurppSubsystem()));
        eventMap.put("stopSlurpp",Commands.runOnce(() -> subsystems.getSlurppSubsystem().stop(), subsystems.getSlurppSubsystem()));
        eventMap.put("floorCollect", MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE));//Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(-0.40), subsystems.getSlurppSubsystem()));


        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Score2Substation", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScore2GuardrailCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("AutoBalance", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScore3SubstationCollect() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L3)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);
        Command collectCone = new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), true)
            .withTimeout(.5);
        Command stowTwo = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);
        Command moveArmL2 = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true);
        Command stowThree = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);
        Command autoBalance = new AutoBalanceCommand((SwerveSubsystemCurrent) subsystems.getSwerveSubsystem());

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne", stowOne);
        eventMap.put("collectCone", collectCone);
        eventMap.put("stowTwo", stowTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);
        eventMap.put("stowThree", stowThree);
        eventMap.put("autoBalance", autoBalance);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Score3SubstationCollect", new PathConstraints(3, 1))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScore3GuardrailCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("AutoBalance", new PathConstraints(1, 3))
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
            PathPlanner.loadPathGroup("TranslationTest", new PathConstraints(3, 1))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getTestCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

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
