package com.cyberknights4911.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystemCurrent;

public class AutoBalanceCommand extends CommandBase {
    
    
    private final SwerveSubsystemCurrent mSwerveSubsystem;
    Translation2d tilt;
    private int counter;


    
    public AutoBalanceCommand (SwerveSubsystemCurrent swerveSubsystem){

      mSwerveSubsystem = swerveSubsystem;
      counter = 0;
      addRequirements(mSwerveSubsystem);

      
    }
    
    @Override
    public void initialize() {

    }

    @Override
  public void execute() {
    tilt = new Translation2d(getRoll(), getPitch()); 
    tilt = tilt.times(Constants.MAX_SPEED);
    mSwerveSubsystem.setTeleopInputs(
                -tilt.getX()*0.008,
                -tilt.getY()*0.008,
                0,
                false,
                false,
                true);
  }

  @Override
  public void end(boolean interrupted) {
    mSwerveSubsystem.setTeleopInputs(
        0.0,
        0.0,
        0.0,
        false,
        true,
        true);  
    }

  @Override
  public boolean isFinished() {
    if (tilt.getNorm()<2){
      counter += 1;
    } else {
      counter = 0;
    }
    return counter >= 10; //TODO: make constants
  }
    

  private double getRoll() {
    return mSwerveSubsystem.getRoll().getDegrees();
  }

  private double getPitch() {
    return mSwerveSubsystem.getPitch().getDegrees();
  }

  
}