package com.cyberknights4911.robot;

import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.control.ButtonAction;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.control.XboxControllerBinding;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.commands.MoveArmCommand;

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

    subsystems.getArmSubsystem().setDefaultCommand(
      new MoveArmCommand(subsystems.getArmSubsystem(), ArmPositions.STOWED)
    );
  }

  private void configureStickBindinges() {
    subsystems.getSwerveSubsystem().setDefaultCommand(
      subsystems.getSwerveSubsystem().createDefaultCommand(controllerBinding)
    );
  }

  private void configureButtonBindings() {
    controllerBinding.triggersFor(ButtonAction.RESET_IMU).onTrue(
      Commands.runOnce(
        () -> subsystems.getSwerveSubsystem().setPose(Constants.ROBOT_STARTING_POSE)
      )
    );

    controllerBinding.triggersFor(ButtonAction.COLLECTOR_BACKWARD).onTrue(
      Commands.runOnce(
        () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
      )
    ).onFalse(
      Commands.runOnce(
        () -> subsystems.getSlurppSubsystem().stop(), subsystems.getSlurppSubsystem()
      )
    );

    controllerBinding.triggersFor(ButtonAction.COLLECTOR_FORWARD).onTrue(
      Commands.runOnce(
        () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
      )
    ).onFalse(
      Commands.runOnce(
        () -> subsystems.getSlurppSubsystem().stop(), subsystems.getSlurppSubsystem()
      )
    );

    controllerBinding.triggersFor(ButtonAction.ARM_L2).onTrue(
      new MoveArmCommand(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_FRONT)
    );
    
    controllerBinding.triggersFor(ButtonAction.ARM_L3).onTrue(
      new MoveArmCommand(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK)
    );
    
    controllerBinding.triggersFor(ButtonAction.STOW).onTrue(
      new MoveArmCommand(subsystems.getArmSubsystem(), ArmPositions.STOWED)
    );

    // Move ONLY safe and tested commands above this line.
    if (true) {
      return;
    }

    controllerBinding.triggersFor(ButtonAction.RESET_WHEELS).onTrue(
      Commands.runOnce(() -> {
        // TODO: reset wheels
      })
    );

    controllerBinding.triggersFor(ButtonAction.CLIMB_LOCKOUT).and(
      controllerBinding.triggersFor(ButtonAction.CLIMB_DEPLOY)
    ).onTrue(
      Commands.runOnce(() -> {
        subsystems.getClimberSubsystem().setExtended(true);
      }, subsystems.getClimberSubsystem())
    );

    controllerBinding.triggersFor(ButtonAction.REAR_COLLECT).onTrue(
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

    controllerBinding.triggersFor(ButtonAction.FRONT_COLLECT_CONE).onTrue(
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

    controllerBinding.triggersFor(ButtonAction.FRONT_COLLECT_CUBE).onTrue(
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

    controllerBinding.triggersFor(ButtonAction.FLOOR_COLLECT).onTrue(
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

    controllerBinding.triggersFor(ButtonAction.BOB_STOW).onTrue(
      Commands.runOnce(
        () -> subsystems.getBobSubsystem().toggleBob(false), subsystems.getBobSubsystem()
      )
    );

    controllerBinding.triggersFor(ButtonAction.BOB_DEPLOY).onTrue(
      Commands.runOnce(
        () -> subsystems.getBobSubsystem().toggleBob(true), subsystems.getBobSubsystem()
      )
    );
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
