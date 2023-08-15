package com.cyberknights4911.robot.model.wham.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class AutoBalanceCommand extends CommandBase {
    private Translation2d tilt;
    private int counter;

    @Override
    public void initialize() {
      counter = 0;
    }

    @Override
    public void execute() {
      tilt = getTilt();
      driveTeleop(new Translation2d(-tilt.getX()*0.003, -tilt.getY()*0.003), 0, false);
    }

    @Override
    public void end(boolean interrupted) {
      driveTeleop(new Translation2d(0.0, 0.0), 0, true);
    }

    @Override
    public boolean isFinished() {
      if (tilt.getNorm() < 2) {
        counter += 1;
      } else {
        counter = 0;
      }
      return counter >= 10; //TODO: make constants
    }

    public abstract void driveTeleop(Translation2d translation, double rotation, boolean fieldRelative);

    public abstract Translation2d getTilt();
}
