// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.cyberknights4911.robot;

import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.control.ButtonAction;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.control.XboxControllerBinding;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;

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

    configureButtonBindings();

    autoCommandChooser = new AutoCommandChooser(subsystems);
  }

  private void configureButtonBindings() {
    subsystems.getSwerveSubsystem().setDefaultCommand(
      subsystems.getSwerveSubsystem().createDefaultCommand(controllerBinding)
    );

    controllerBinding.triggerFor(ButtonAction.RESET_IMU).onTrue(
      Commands.runOnce(
        () -> subsystems.getSwerveSubsystem().setPose(Constants.ROBOT_STARTING_POSE)
      )
    );

    controllerBinding.triggerFor(ButtonAction.RELEASE_PIECE).onTrue(
      Commands.runOnce(() -> subsystems.getSlurppSubsystem().spit())
    );

    controllerBinding.triggerFor(ButtonAction.COLLECT_CUBE).onTrue(
      Commands.sequence(
        Commands.runOnce(
          () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
        ),
        Commands.runOnce(
          () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
        )
      )
    );

    controllerBinding.triggerFor(ButtonAction.COLLECT_CONE).onTrue(
      Commands.sequence(
        Commands.runOnce(
          () -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()
        ),
        Commands.runOnce(
          () -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem()
        )
      )
    );

    controllerBinding.triggerFor(ButtonAction.CLIMB_WHEEL_LOCK).onTrue(
      Commands.runOnce(() -> {
        // TODO: lock wheels
      })
    );

    controllerBinding.triggerFor(ButtonAction.RESET_WHEELS).onTrue(
      Commands.runOnce(() -> {
        // TODO: reset wheels
      })
    );

    controllerBinding.triggerFor(ButtonAction.ARM_L2).onTrue(
      Commands.runOnce(()-> {
        subsystems.getArmSubsystem().setDesiredPosition(ArmPositions.CUBE_LEVEL_2);
      }, subsystems.getArmSubsystem())
    );
    
    controllerBinding.triggerFor(ButtonAction.ARM_L3).onTrue(
      Commands.runOnce(()-> {
        subsystems.getArmSubsystem().setDesiredPosition(ArmPositions.CUBE_LEVEL_3);
      }, subsystems.getArmSubsystem())
    );

    controllerBinding.triggerFor(ButtonAction.CLIMB_LOCKOUT).and(
      controllerBinding.triggerFor(ButtonAction.CLIMB_DEPLOY).onTrue(
        Commands.runOnce(() ->
        subsystems.getClimberSubsystem().setExtended(true), subsystems.getClimberSubsystem())
      )
    );
    
    controllerBinding.triggerFor(ButtonAction.STOW).onTrue(
      Commands.runOnce(() -> {
        // TODO: stow the arm & claw
      }, subsystems.getArmSubsystem())
    );

    controllerBinding.triggerFor(ButtonAction.REAR_COLLECT).onTrue(
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

    controllerBinding.triggerFor(ButtonAction.FRONT_COLLECT).onTrue(
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

    controllerBinding.triggerFor(ButtonAction.FLOOR_COLLECT).onTrue(
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

    controllerBinding.triggerFor(ButtonAction.BOB_STOW).onTrue(
      Commands.runOnce(
        () -> subsystems.getBobSubsystem().toggleBob(false), subsystems.getBobSubsystem()
      )
    );

    controllerBinding.triggerFor(ButtonAction.BOB_DEPLOY).onTrue(
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
