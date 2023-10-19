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
import com.cyberknights4911.robot.model.wham.slurpp.SlurppSubsystem;
import com.cyberknights4911.robot.model.wham.slurpp.CollectConfig.GamePiece;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlockPartyAutos implements AutoCommandHandler {

    private final ArmSubsystem armSubsystem;
    private final SlurppSubsystem slurppSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final LoggedDashboardChooser<Command> chooser;
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<Pose2d> resetPose;
    private final PIDConstants translationConstants;
    private final PIDConstants rotationConstants;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    
    private Command currentAutoCommand;

    public BlockPartyAutos(ArmSubsystem armSubsystem, SlurppSubsystem slurppSubsystem,
            SwerveSubsystem swerveSubsystem) {
        this.armSubsystem = armSubsystem;
        this.slurppSubsystem = slurppSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        
        // PID constants to correct for translation error (used to create the X and Y PID controllers)
        translationConstants = new PIDConstants(4.3, 0, 0.0);

        // PID constants to correct for rotation error (used to create the rotation controller)
        rotationConstants = new PIDConstants(2.0, 0.0, 0.0);

        poseSupplier = swerveSubsystem::getPose;
        resetPose = swerveSubsystem::setPose;
        kinematics = swerveSubsystem.getKinematics();
        outputModuleStates = swerveSubsystem::setSwerveModuleStates;

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

    private SwerveAutoBuilder swerveAutoBuilder(Map<String, Command> eventMap) {
        return new SwerveAutoBuilder(
            poseSupplier,
            resetPose,
            kinematics,
            translationConstants,
            rotationConstants,
            outputModuleStates,
            eventMap,
            swerveSubsystem);
    }

    private Command pathCommand(Map<String, Command> eventMap, String pathName, double maxSpeed, double maxAcceleration) {
        return swerveAutoBuilder(eventMap)
            .fullAuto(PathPlanner.loadPath(pathName, maxSpeed, maxAcceleration));
    }

    private Command scoreAndStow() {
        return scoreHighOnly()
            .andThen(armSubsystem.createArmCommand(slurppSubsystem, ArmPositions.STOWED));
    }

    private Command scoreExit() {
        Map<String, Command> events = new HashMap<>();

        Command pathCommand = pathCommand(events, "pathname", 3, 1.5);
        Command driveAndStow = armSubsystem.createArmCommand(slurppSubsystem, ArmPositions.STOWED)
            .alongWith(pathCommand);

        return scoreHighOnly().andThen(driveAndStow);
    }

    private Command scoreBalance() {
        return Commands.runOnce(() -> {});
    }

    private Command scoreCollect() {
        return Commands.runOnce(() -> {});
    }

    private Command scoreCollectBalance() {
        return Commands.runOnce(() -> {});
    }

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
