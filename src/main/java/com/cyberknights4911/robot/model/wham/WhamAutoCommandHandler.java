package com.cyberknights4911.robot.model.wham;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.cyberknights4911.robot.auto.AutoCommandHandler;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
import com.cyberknights4911.robot.model.wham.arm.ArmPositions;
import com.cyberknights4911.robot.model.wham.arm.ArmSubsystem;
import com.cyberknights4911.robot.model.wham.arm.MoveArmMotionMagicCommand;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppCommand;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class WhamAutoCommandHandler implements AutoCommandHandler {

    private final ArmSubsystem armSubsystem;
    private final SlurppSubsystem slurppSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final LoggedDashboardChooser<Command> loggedDashboardChooser;
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<Pose2d> resetPose;
    private final PIDConstants translationConstants;
    private final PIDConstants rotationConstants;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<SwerveModuleState[]> outputModuleStates;

    private Command currentAutoCommand;

    public WhamAutoCommandHandler(ArmSubsystem armSubsystem, SlurppSubsystem slurppSubsystem, SwerveSubsystem swerveSubsystem) {
        this.armSubsystem = armSubsystem;
        this.slurppSubsystem = slurppSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        loggedDashboardChooser = new LoggedDashboardChooser<Command>("Auto Routine");
        
        // PID constants to correct for translation error (used to create the X and Y PID controllers)
        translationConstants = new PIDConstants(4.3, 0, 0.0);

        // PID constants to correct for rotation error (used to create the rotation controller)
        rotationConstants = new PIDConstants(2.0, 0.0, 0.0);

        poseSupplier = swerveSubsystem::getPose;
        resetPose = swerveSubsystem::setPose;
        kinematics = swerveSubsystem.getKinematics();
        outputModuleStates = swerveSubsystem::setSwerveModuleStates;


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
        loggedDashboardChooser.addDefaultOption("Score and AutoBalance", getAutoBalanceCommand());
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
        // loggedDashboardChooser.addOption("Test", getTestCommand());

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

        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(2.0))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);
        Command autoBalance = swerveSubsystem.createAutobalanceCommand();

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne",stowOne);
        eventMap.put("autoBalance", autoBalance);


        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("AutoBalance", new PathConstraints(3, 1.5))
        );

        return autoCommand;
    }

    private Command getNothingCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        

        Command autoCommand = Commands.none();

        return autoCommand;
    }

    private Command getScoreHighCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(2.0))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne",stowOne);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreHigh", new PathConstraints(0, 0))
        );

        return autoCommand;
    }

    private Command getScoreLowCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, true)
            .withTimeout(.5);

        eventMap.put("scoreConeOne", scoreConeOne);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreLow", new PathConstraints(0, 0))
        );

        return autoCommand;
    }


    private Command getBalanceScoreLeaveCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(2.0))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);
        Command autoBalance = swerveSubsystem.createAutobalanceCommand();

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne",stowOne);
        eventMap.put("autoBalance", autoBalance);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("Balance and Leave", new PathConstraints(4, 2))
        );

        return autoCommand;
    }

    private Command getScoreCollectBalanceSubstationCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_BACK)
            // .andThen(Commands.waitSeconds(.1))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(.03));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(slurppSubsystem, 0.4, armSubsystem, false));
        Command stow = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED)
            .andThen(new SlurppCommand(slurppSubsystem, 0.0, armSubsystem, false));
        Command autoBalance = swerveSubsystem.createAutobalanceCommand();

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("stow", stow);
        eventMap.put("autoBalance", autoBalance);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreSubstationCollectAutobalance", new PathConstraints(3, 1.2))
        );

        return autoCommand;
    }

    //Tested by rileyTest
    private Command getScore2BalanceCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_BACK)
            // .andThen(Commands.waitSeconds(.1))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(.1));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(slurppSubsystem, 0.4, armSubsystem, false));
        Command moveArmL2 = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, false)
            .andThen(Commands.waitSeconds(.1));
        Command stow = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);
        Command autoBalance = swerveSubsystem.createAutobalanceCommand();
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);
        eventMap.put("stow", stow);
        eventMap.put("autoBalance", autoBalance);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("Score2SubstationBalance", new PathConstraints(3, 1))
        );

        return autoCommand;
    }

    private Command getScoreLeaveGuardCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne", stowOne);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreLeaveGuard", new PathConstraints(1, 3))
        );

        return autoCommand;
    }

    private Command getScoreLeaveSubstationCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);

        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne", stowOne);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("ScoreLeaveSubstation", new PathConstraints(3, 1))
        );

        return autoCommand;
    }

    private Command getScore2SubstationCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_BACK)
            // .andThen(Commands.waitSeconds(.1))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(.03));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(slurppSubsystem, 0.4, armSubsystem, false));
        Command moveArmL2 = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, false)
            .andThen(Commands.waitSeconds(1))
            .andThen(MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED))
            .andThen(new SlurppCommand(slurppSubsystem, 0, armSubsystem, false));
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("Score2Substation", new PathConstraints(3, 1))
        );

        return autoCommand;
    }

    private Command getScore2GuardrailCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = new SlurppCommand(slurppSubsystem, .4, armSubsystem, true)
            .andThen(MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_BACK))
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true))
            .andThen(Commands.waitSeconds(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);
        Command collectCone = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .alongWith(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, true))
            .withTimeout(3);
        Command stowTwo = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);
        Command moveArmL2 = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .withTimeout(.5);
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("stowOne", stowOne);
        eventMap.put("collectCone", collectCone);
        eventMap.put("stowTwo", stowTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("Score2Guard", new PathConstraints(1, 3))
        );

        return autoCommand;
    }

    private Command getScore3Substation() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_BACK)
            // .andThen(Commands.waitSeconds(.1))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(.1));
        Command collectConeTwo = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(1))
            .andThen(new SlurppCommand(slurppSubsystem, 0.4, armSubsystem, false));
        Command moveArmL2 = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L2);
        Command scoreConeTwo = new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, false)
            .andThen(Commands.waitSeconds(.1));
        Command collectConeThree = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, true))
            .andThen(Commands.waitSeconds(2))
            .andThen( MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED));
        Command scoreConeThree = new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .andThen(Commands.waitSeconds(.5));
        
        eventMap.put("scoreConeOne", scoreConeOne);
        eventMap.put("collectConeTwo", collectConeTwo);
        eventMap.put("moveArmL2", moveArmL2);
        eventMap.put("scoreConeTwo", scoreConeTwo);
        eventMap.put("collectConeThree", collectConeThree);
        eventMap.put("scoreConeThree", scoreConeThree);

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("Score3Substation", new PathConstraints(3, 1))
        );

        return autoCommand;
    }
    
    private Command getRileyTest() {
        return MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, false))
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(slurppSubsystem, 0.0, armSubsystem, false));
    }

    private Command getScore3GuardrailCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();
        Command scoreConeOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L3)
            .andThen(Commands.waitSeconds(.5))
            .andThen(new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .withTimeout(.5));
        Command stowOne = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);
        Command collectCone = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, true))
            .withTimeout(2);
        Command stowTwo = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);
        Command scoreConeTwo = new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
            .withTimeout(.5);

        Command collectConeTwo = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE)
            .andThen(new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, true))
            .withTimeout(2);
        Command stowThree = MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED);
        Command scoreConeThree = new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true)
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
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("Score3Guard", new PathConstraints(1, 3))
        );

        return autoCommand;
    }

    private Command getRotationTestCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("RotationTest", new PathConstraints(1, 3))
        );

        return autoCommand;
    }

    private Command getTranslationTestCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("TranslationTest", new PathConstraints(3, 1))
        );

        return autoCommand;
    }

    private Command getTestCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        Command autoCommand = createSwerveAutoBuilder(
            eventMap,
            swerveSubsystem
        ).fullAuto(
            PathPlanner.loadPathGroup("Test", new PathConstraints(1, 3))
        );

        //TODO: We might need to call a stop on this or set something to stop the robot after it runs.
        return autoCommand;
    }

    @Override
    public void startCurrentAutonomousCommand() {
        stopCurrentAutonomousCommand();
        currentAutoCommand = loggedDashboardChooser.get();
        if (currentAutoCommand != null) {
            currentAutoCommand.schedule();
        }
    }

    @Override
    public void stopCurrentAutonomousCommand() {
        if (currentAutoCommand != null) {
            currentAutoCommand.cancel();
        }
    }
}
