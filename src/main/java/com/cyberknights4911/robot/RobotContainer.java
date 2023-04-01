package com.cyberknights4911.robot;

import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.control.ButtonAction;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.control.XboxControllerBinding;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;
import com.cyberknights4911.robot.commands.CANCoderResetCommand;
import com.cyberknights4911.robot.commands.MoveArmMotionMagicCommand;
import com.cyberknights4911.robot.commands.SlurppCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED)
    );

    subsystems.getSlurppSubsystem().setDefaultCommand(
      Commands.runOnce(() -> {
        subsystems.getSlurppSubsystem().holdCurrentPosition();
      }, subsystems.getSlurppSubsystem())
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

    controllerBinding.triggersFor(ButtonAction.SLURPP_BACKWARD_FAST).whileTrue(
      new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true));

    controllerBinding.triggersFor(ButtonAction.SLURPP_BACKWARD_SLOW).whileTrue(
      new SlurppCommand(subsystems.getSlurppSubsystem(), -0.4, subsystems.getArmSubsystem(), true));

    controllerBinding.triggersFor(ButtonAction.SLURPP_FORWARD_FAST).whileTrue(
        new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), true));

    controllerBinding.triggersFor(ButtonAction.SLURPP_FORWARD_SLOW).whileTrue(
      new SlurppCommand(subsystems.getSlurppSubsystem(), 0.4, subsystems.getArmSubsystem(), true));

    controllerBinding.triggersFor(ButtonAction.COLLECT_SINGLE_SUBSTATION_FRONT).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SINGLE_SUBSTATION_FRONT));
      
    controllerBinding.triggersFor(ButtonAction.COLLECT_SUBSTATION_FRONT).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_FRONT));

    controllerBinding.triggersFor(ButtonAction.COLLECT_SUBSTATION_BACK).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK));
    
    controllerBinding.triggersFor(ButtonAction.COLLECT_FLOOR_BACK_CONE).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_BACK_CONE));
    
    controllerBinding.triggersFor(ButtonAction.COLLECT_FLOOR_BACK_CUBE).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_BACK_CUBE));
  
    controllerBinding.triggersFor(ButtonAction.COLLECT_FLOOR_FRONT_CONE).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE));

    controllerBinding.triggersFor(ButtonAction.COLLECT_FLOOR_FRONT_CUBE).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CUBE));

    controllerBinding.triggersFor(ButtonAction.STOW).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED));

    controllerBinding.triggersFor(ButtonAction.SCORE_L2).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L2));

    controllerBinding.triggersFor(ButtonAction.SCORE_L3).onTrue(
      MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L3));
    
    SmartDashboard.putData("Arm CANCoder Reset", new CANCoderResetCommand(subsystems.getArmSubsystem()));
    
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
