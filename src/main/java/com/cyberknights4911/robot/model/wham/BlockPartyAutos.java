package com.cyberknights4911.robot.model.wham;

import java.util.HashMap;
import java.util.Map;
import com.cyberknights4911.robot.auto.PathPlannerCommandFactory;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.cyberknights4911.robot.auto.AutoCommandHandler;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
import com.cyberknights4911.robot.model.wham.arm.ArmPositions;
import com.cyberknights4911.robot.model.wham.arm.ArmSubsystem;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppSubsystem;
import com.cyberknights4911.robot.model.wham.slurpp.CollectConfig.GamePiece;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlockPartyAutos implements AutoCommandHandler {

    private final PathPlannerCommandFactory pathFactory;
    private final ArmSubsystem armSubsystem;
    private final SlurppSubsystem slurppSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final LoggedDashboardChooser<Command> chooser;
    
    private Command currentAutoCommand;

    public BlockPartyAutos(ArmSubsystem armSubsystem, SlurppSubsystem slurppSubsystem,
            SwerveSubsystem swerveSubsystem) {
        this.armSubsystem = armSubsystem;
        this.slurppSubsystem = slurppSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        // PID constants to correct for translation error (used to create the X and Y PID controllers)
        PIDConstants translationConstants = new PIDConstants(4.3, 0, 0.0);
        // PID constants to correct for rotation error (used to create the rotation controller)
        PIDConstants rotationConstants = new PIDConstants(2.0, 0.0, 0.0);

        pathFactory = new PathPlannerCommandFactory(
            swerveSubsystem::getPose,
            swerveSubsystem::setPose,
            swerveSubsystem.getKinematics(),
            swerveSubsystem::setSwerveModuleStates,
            translationConstants,
            rotationConstants,
            swerveSubsystem
        );

        chooser = new LoggedDashboardChooser<Command>("Auto Routine");
        chooser.addDefaultOption("Nothing", Commands.runOnce(() -> {}));
        chooser.addOption("Score", scoreAndStow());
        chooser.addOption("Score, exit", scoreExit());
        chooser.addOption("Score, balance", scoreBalance());
        chooser.addOption("Score, collect", scoreCollect());
        chooser.addOption("Score, collect, balance", scoreCollectBalance());
        chooser.addOption("Score, collect, score, exit", scoreCollectScoreExit());
    }

    /**
     * This is NOT intended to be used as an auto. It scores a cone on the highest row and leaves
     * the arm extended. Other autos can use this as a building block in order to begin moving
     * and stowing the arm at the same time.
     */
    private Command scoreHighOnly() {
        // adjust these delays until cone consistently scores, but only just
        double armMotionWaitDelay = 2;
        double slurppScoreDelay = .3;
        return Commands.runOnce(() -> slurppSubsystem.setGamePiece(GamePiece.CONE))
            .andThen(armSubsystem.createArmCommand(slurppSubsystem, ArmPositions.STOWED))
            .andThen(slurppSubsystem.createRetainCommand())
            .andThen(armSubsystem.createArmCommand(slurppSubsystem, ArmPositions.SCORE_L3))
            .andThen(Commands.waitSeconds(armMotionWaitDelay))
            .andThen(slurppSubsystem.createScoreCommand())
            .andThen(Commands.waitSeconds(slurppScoreDelay))
            .andThen(slurppSubsystem.createStopCommand());
    }

    /**
     * Builds a command that follows a path.
     */
    private Command pathCommand(Map<String, Command> eventMap, String pathName, double maxSpeed,
            double maxAcceleration) {
        return pathFactory.createAutoCommand(
            PathPlanner.loadPath(pathName, maxSpeed, maxAcceleration),
            eventMap
        );
    }

    /**
     * Scores a high cone and then stows the arm. That's it.
     */
    private Command scoreAndStow() {
        return scoreHighOnly()
            .andThen(armSubsystem.createArmCommand(slurppSubsystem, ArmPositions.STOWED));
    }

    /**
     * Scores a high cone, stows the arm and exits community.
     */
    private Command scoreExit() {
        Map<String, Command> events = new HashMap<>();

        Command pathCommand = pathCommand(events, "pathname", 3, 1.5);

        // This command stows the arm as the robot begins moving
        Command driveAndStow = armSubsystem.createArmCommand(slurppSubsystem, ArmPositions.STOWED)
            .alongWith(pathCommand);

        return scoreHighOnly().andThen(driveAndStow);
    }

    /**
     * Scores a high cone. Stows the arm while exiting community. Finishes with an autobalance.
     */
    private Command scoreBalance() {
        return Commands.runOnce(() -> {});
    }

    /**
     * Scores a high cone. Stows the arm while exiting community. Finishes by collecting a second cone.
     */
    private Command scoreCollect() {
        return Commands.runOnce(() -> {});
    }

    /**
     * Scores a high cone. Stows the arm while exiting community. Collects a second cone. Scores
     * second cone mid. Stows the arm.
     */
    private Command scoreCollectBalance() {
        return Commands.runOnce(() -> {});
    }

    /**
     * Scores a high cone. Stows the arm while exiting community. Collects a second cone. Scores
     * second cone mid. Stows the arm while exiting community.
     */
    private Command scoreCollectScoreExit() {
        return Commands.runOnce(() -> {});
    }

    @Override
    public void startCurrentAutonomousCommand() {
        stopCurrentAutonomousCommand();
        currentAutoCommand = chooser.get();
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
