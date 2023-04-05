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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


        //Do Nothing
        //Score High
        //Score Low
        //Score High and Leave Guard Side
        //Score High and Leave Substation
        //Score 2 Guard Side
        //Score 2 Substation Side
        //Score 3 Guard Side
        //Score 3 Substation Side
        //Score 2 AutoBalance
        //Score Leave and AutoBalance
        //Score and AutoBalance
        //Score Collect and Autobalance
        loggedDashboardChooser.addDefaultOption("Do Nothing", getNothingCommand());
        loggedDashboardChooser.addOption("Score and AutoBalance", getAutoBalanceCommand());
        loggedDashboardChooser.addOption("Score Leave and AutoBalance", getBalanceScoreLeaveCommand());
        loggedDashboardChooser.addOption("Score 1.5 Substation and Autobalance", getScoreCollectBalanceSubstationCommand());
        loggedDashboardChooser.addOption("Score 2 and AutoBalance", getScore2BalanceCommand());
        loggedDashboardChooser.addOption("Score High", getScoreHighCommand());
        loggedDashboardChooser.addOption("Score Low", getScoreLowCommand());
        loggedDashboardChooser.addOption("Score And Leave", getScoreHighCommand());
        loggedDashboardChooser.addOption("Score High and Leave Guard Side", getScoreLeaveGuardCommand());
        loggedDashboardChooser.addOption("Score High and Leave Substation Side", getScoreLeaveSubstationCommand());
        loggedDashboardChooser.addOption("Score 2 Substation Side", getScore2SubstationCommand());
        loggedDashboardChooser.addOption("Score 2 Guard Side", getScore2GuardrailCommand());
        loggedDashboardChooser.addOption("Score 3 Substation Side", getScore3Substation());
        loggedDashboardChooser.addOption("Score 3 Guard Side", getScore3GuardrailCommand());

        loggedDashboardChooser.addOption("Riley Test", getRileyTest());
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
            true,
            driveRequirements
        );
    }

    private Command getAutoBalanceCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
        // .andThen(Commands.waitSeconds(.1))
        .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false))
        .andThen(Commands.waitSeconds(.03));
        Command stow = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED)
        .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.0, subsystems.getArmSubsystem(), false));
    Command autoBalance = new AutoBalanceCommand((SwerveSubsystemCurrent) subsystems.getSwerveSubsystem());

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne",stow);
        eventMap.put("autoBalance", autoBalance);


        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("AutoBalance", new PathConstraints(3, 1.5))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getNothingCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        

        Command autoCommand = Commands.none();

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScoreHighCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(.3));
        Command stow = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.0, subsystems.getArmSubsystem(), false));

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne",stow);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreHigh", new PathConstraints(0, 0))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScoreLowCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true)
            .withTimeout(.5);

        eventMap.put("scoreConeOne", scoreConeOne);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreLow", new PathConstraints(0, 0))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }


    private Command getBalanceScoreLeaveCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
        // .andThen(Commands.waitSeconds(.1))
        .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false))
        .andThen(Commands.waitSeconds(.03));
        Command stow = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED)
        .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.0, subsystems.getArmSubsystem(), false));
    Command autoBalance = new AutoBalanceCommand((SwerveSubsystemCurrent) subsystems.getSwerveSubsystem());

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne",stow);
        eventMap.put("autoBalance", autoBalance);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Balance and Leave", new PathConstraints(1, 1))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScoreCollectBalanceSubstationCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
            // .andThen(Commands.waitSeconds(.1))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(.03));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.4, subsystems.getArmSubsystem(), false));
        Command stow = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.0, subsystems.getArmSubsystem(), false));
        Command autoBalance = new AutoBalanceCommand((SwerveSubsystemCurrent) subsystems.getSwerveSubsystem());

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("stow", stow);
        eventMap.put("autoBalance", autoBalance);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreSubstationCollectAutobalance", new PathConstraints(3, 1.2))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScore2BalanceCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
            // .andThen(Commands.waitSeconds(.1))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(.1));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.4, subsystems.getArmSubsystem(), false));
        Command moveArmL2 = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false)
            .andThen(Commands.waitSeconds(.1));
        Command stow = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);
        Command autoBalance = new AutoBalanceCommand((SwerveSubsystemCurrent) subsystems.getSwerveSubsystem());
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);
        eventMap.put("stow", stow);
        eventMap.put("autoBalance", autoBalance);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Score2SubstationBalance", new PathConstraints(3, 1))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScoreLeaveGuardCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne", stowOne);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreLeaveGuard", new PathConstraints(1, 3))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScoreLeaveSubstationCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne", stowOne);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreLeaveSubstation", new PathConstraints(2, 1))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScore2SubstationCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(.03));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.4, subsystems.getArmSubsystem(), false));
        Command moveArmL2 = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false)
            .andThen(Commands.waitSeconds(1))
            .andThen(MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0, subsystems.getArmSubsystem(), false));
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Score2Substation", new PathConstraints(3, 1))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScore2GuardrailCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(.03));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.4, subsystems.getArmSubsystem(), false));
        Command moveArmL2 = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false)
            .andThen(Commands.waitSeconds(1))
            .andThen(MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0, subsystems.getArmSubsystem(), false));
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Score2Guard", new PathConstraints(3, 1))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }

    private Command getScore3Substation() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
            // .andThen(Commands.waitSeconds(.1))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(.1));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.4, subsystems.getArmSubsystem(), false));
        Command moveArmL2 = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), false)
            .andThen(Commands.waitSeconds(.1));
        Command collectConeThree = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), true))
            .andThen(Commands.waitSeconds(2))
            .andThen( MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED));
        Command scoreConeThree = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true)
            .andThen(Commands.waitSeconds(.5));
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);
        eventMap.put("collectConeThree", collectConeThree);
        eventMap.put("scoreConeThree", scoreConeThree);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Score3Substation", new PathConstraints(3, 1))
        );

        return new InstantCommand(
            () -> subsystems.getSwerveSubsystem().initForPathFollowing()
        ).andThen(autoCommand);
    }
    
    private Command getRileyTest() {
        return MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), false))
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.0, subsystems.getArmSubsystem(), false));
    }

    private Command getScore3GuardrailCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);
        Command collectCone = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), true))
            .withTimeout(2);
        Command stowTwo = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);
        Command scoreConeTwo = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true)
            .withTimeout(.5);

        Command collectConeTwo = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), true))
            .withTimeout(2);
        Command stowThree = MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED);
        Command scoreConeThree = new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true)
            .withTimeout(.5);
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne", stowOne);
        eventMap.put("collectCone", collectCone);
        eventMap.put("stowTwo", stowTwo);
        eventMap.put("scoreConeTwo", scoreConeTwo);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("stowThree", stowThree);
        eventMap.put("scoreConeThree", scoreConeThree);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            subsystems.getSwerveSubsystem()
        ).fullAuto(
            PathPlanner.loadPathGroup("Score3Guard", new PathConstraints(1, 3))
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
