package com.cyberknights4911.robot;

import com.cyberknights4911.robot.model.RobotModel;
import com.cyberknights4911.robot.model.quickdrop.QuickDrop;
import com.cyberknights4911.robot.subsystems.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Subsystems subsystems;
  private final RobotModel robotModel;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    subsystems = new Subsystems();
    robotModel = new QuickDrop();

    robotModel.applyDefaultCommands(subsystems);
    robotModel.applyButtonActions(subsystems);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return robotModel.geAutoCommandChooser().getAutonomousCommand();
  }
}
