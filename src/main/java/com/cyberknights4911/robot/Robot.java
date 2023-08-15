package com.cyberknights4911.robot;

import java.util.function.Supplier;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import com.cyberknights4911.robot.logging.RobotLogger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public final class Robot extends LoggedRobot {
  private final Supplier<RobotStateListener> listenerSupplier;
  private final RobotLogger robotLogger;

  private RobotStateListener listener;

  public Robot(Supplier<RobotStateListener> listenerSupplier) {
    super();
    this.listenerSupplier = listenerSupplier;
    robotLogger = new RobotLogger(Logger.getInstance());
  }

  @Override
  public void robotInit() {
    robotLogger.startLogging(this);
    listener = listenerSupplier.get();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    if (listener != null) {
      listener.onAutonomousInit(this);
    }
  }

  @Override
  public void autonomousExit() {
    if (listener != null) {
      listener.onAutonomousExit(this);
    }
  }

  @Override
  public void teleopInit() {
    if (listener != null) {
      listener.onTeleopInit(this);
    }
  }

  @Override
  public void teleopExit() {
    if (listener != null) {
      listener.onTeleopExit(this);
    }
  }
}
