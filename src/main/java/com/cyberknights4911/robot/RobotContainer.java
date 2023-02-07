  // `

package com.cyberknights4911.robot;

import com.cyberknights4911.robot.commands.DefaultSwerveCommand;
import com.cyberknights4911.robot.commands.MoveHoodCommand;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.control.ButtonAction;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.control.StickAction;
import com.cyberknights4911.robot.control.XboxControllerBinding;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystem;
import com.cyberknights4911.robot.subsystems.hood.HoodSubsystem.HoodPositions;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.HashMap;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Subsystems subsystems;
  private final ControllerBinding controllerBinding;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    subsystems = new Subsystems();
    controllerBinding = new XboxControllerBinding();

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    subsystems.getSwerveSubsystem().convertCancoderToFX();

    subsystems.getSwerveSubsystem().setDefaultCommand(
      new DefaultSwerveCommand(
        subsystems.getSwerveSubsystem(),
        controllerBinding.supplierFor(StickAction.FORWARD),
        controllerBinding.supplierFor(StickAction.STRAFE),
        controllerBinding.supplierFor(StickAction.ROTATE)
      )
    );

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.RESET_IMU)) {
      trigger.onTrue(
        Commands.runOnce(
          () -> subsystems.getSwerveSubsystem().setRobotPosition(Constants.ROBOT_STARTING_POSE)
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.RELEASE)) {
      trigger.onTrue(
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().spit())
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.COLLECT)) {
      trigger.onTrue(
        Commands.sequence(
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
          ),
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
          )
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.COLLECT)) {
      trigger.onTrue(
        Commands.sequence(
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
          ),
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
          )
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.CLIMB_WHEEL_LOCK)) {
      trigger.onTrue(
        Commands.runOnce(() -> {
          // TODO: lock wheels
        })
      );
    }

   for (Trigger trigger :  controllerBinding.triggerFor(ButtonAction.RESET_WHEELS)) {
    trigger.onTrue(
        Commands.runOnce(() -> {
          // TODO: reset wheels
        })
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.ARM_L2)) {
      trigger.onTrue(
        Commands.runOnce(()-> {
          subsystems.getArmSubsystem().setDesiredPosition(ArmPositions.CUBE_LEVEL_2);
        }, subsystems.getArmSubsystem())
      );
    }
    
    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.ARM_L3)) {
      trigger.onTrue(
        Commands.runOnce(()-> {
          subsystems.getArmSubsystem().setDesiredPosition(ArmPositions.CUBE_LEVEL_3);
        }, subsystems.getArmSubsystem())
      );
    }

    for (Trigger trigger1 : controllerBinding.triggerFor(ButtonAction.CLIMB_LOCKOUT)) {
      for (Trigger trigger2 : controllerBinding.triggerFor(ButtonAction.CLIMB_DEPLOY)) {
        trigger1.and(trigger2).onTrue(
          Commands.runOnce(() ->
            subsystems.getClimberSubsystem().setExtended(true), subsystems.getClimberSubsystem())
          );
      }
    }
    
    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.STOW)) {
      trigger.onTrue(
        Commands.runOnce(() -> {
          // TODO: stow the arm & claw
        }, subsystems.getArmSubsystem())
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.REAR_COLLECT)) {
      trigger.onTrue(
        Commands.sequence(
          Commands.parallel(
            Commands.runOnce(
              () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
            ),
            Commands.runOnce(() -> {
              // TODO move to rear collect
            }, subsystems.getArmSubsystem())
          ),
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
          )
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.FRONT_COLLECT_CONE)) {
      trigger.onTrue(
        Commands.sequence(
          Commands.parallel(
            Commands.runOnce(
              () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
            ),
            Commands.runOnce(() -> {
              // TODO move to front collect
            }, subsystems.getArmSubsystem())
          ),
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
          )
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.FRONT_COLLECT_CUBE)) {
      trigger.onTrue(
        Commands.sequence(
          Commands.parallel(
            Commands.runOnce(
              () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
            ),
            Commands.runOnce(() -> {
              // TODO move to front collect
            }, subsystems.getArmSubsystem())
          ),
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
          )
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.FLOOR_COLLECT)) {
      trigger.onTrue(
        Commands.sequence(
          Commands.parallel(
            Commands.runOnce(
              () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
            ),
            Commands.runOnce(() -> {
              // TODO move to floor collect
            }, subsystems.getArmSubsystem())
          ),
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
          )
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.BOB_STOW)) {
      trigger.onTrue(
        Commands.runOnce(
          () -> subsystems.getBobSubsystem().toggleBob(false), subsystems.getBobSubsystem()
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.BOB_DEPLOY)) {
      trigger.onTrue(
        Commands.runOnce(
          () -> subsystems.getBobSubsystem().toggleBob(true), subsystems.getBobSubsystem()
        )
      );
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // This will load the file "Test.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test", new PathConstraints(4, 3));
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("hood", new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.H2));
    eventMap.put("hoodStow", new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.STOWED));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        subsystems.getSwerveSubsystem()::getPose, // Pose2d supplier
        subsystems.getSwerveSubsystem()::setRobotPosition, // Pose2d consumer, used to reset odometry at the beginning of auto
        subsystems.getSwerveSubsystem().getmKinematics(), // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        subsystems.getSwerveSubsystem()::setSwerveModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        subsystems.getSwerveSubsystem() // The drive subsystem. Used to properly set the requirements of path following commands
    );

    Command fullAuto = new InstantCommand(() -> subsystems.getSwerveSubsystem().setState(SwerveSubsystem.ControlState.PATH_FOLLOWING))
        .andThen(autoBuilder.fullAuto(pathGroup));

    //TODO: We might need to call a stop on this or set something to stop the robot after it runs.
    return fullAuto;
  }
}
