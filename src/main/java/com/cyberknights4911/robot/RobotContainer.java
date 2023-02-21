  // `

package com.cyberknights4911.robot;

import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.control.ButtonAction;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.control.XboxControllerBinding;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Subsystems subsystems;
  private final ControllerBinding controllerBinding;
  private final AutoCommandChooser autoCommandChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    subsystems = new Subsystems();

    controllerBinding = new XboxControllerBinding();

    autoCommandChooser = new AutoCommandChooser(subsystems);

    configureStickBindinges();
    configureButtonBindings();
  }

  private void configureStickBindinges() {
    subsystems.getSwerveSubsystem().setDefaultCommand(
      subsystems.getSwerveSubsystem().createDefaultCommand(controllerBinding)
    );
  }

  private void configureButtonBindings() {
    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.RESET_IMU)) {
      trigger.onTrue(
        Commands.runOnce(
          () -> subsystems.getSwerveSubsystem().setPose(Constants.ROBOT_STARTING_POSE)
        )
      );
    }

    // Move ONLY safe and tested commands above this line.
    if (true) {
      return;
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.COLLECTOR_BACKWARD)) {
      trigger.onTrue(
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().spit())
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.COLLECTOR_FORWARD)) {
      trigger.onTrue(
        Commands.sequence(
          Commands.runOnce(
            () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
          )
        )
      );
    }

    for (Trigger trigger : controllerBinding.triggerFor(ButtonAction.COLLECTOR_FORWARD)) {
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
    return autoCommandChooser.getCurrentCommand();
  }
}
