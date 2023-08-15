package com.cyberknights4911.robot.logging;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import com.cyberknights4911.robot.BuildConstants;
import edu.wpi.first.wpilibj.RobotBase;

public final class RobotLogger {
  /*
   * Used to control whether to log to NetworkTables. This can be resource intensive so it should
   * be disabled for competitions.
   */
  private static final boolean ENABLE_NT_LOGGING = true;

  private final Logger logger;

  public RobotLogger(Logger logger) {
    this.logger = logger;
  }

  public void startLogging(LoggedRobot robot) {
    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    if (RobotBase.isReal()) {
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      if (ENABLE_NT_LOGGING) {
        logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      }
    } else {
      robot.setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    logger.start();
  }
}
