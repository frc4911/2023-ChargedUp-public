package com.cyberknights4911.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystemCurrent;

public class AutoBalanceCommand extends CommandBase {
    
    
    private final SwerveSubsystemCurrent mSwerveSubsystem;
    Translation2d tilt;


    
    public AutoBalanceCommand (SwerveSubsystemCurrent swerveSubsystem){

      mSwerveSubsystem = swerveSubsystem;
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
                tilt.getX()*0.01,
                tilt.getY()*0.01,
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
    System.out.println(tilt.getNorm());
    return (tilt.getNorm()<2); //TODO: make constants
  }
    

  private double getRoll() {
    return mSwerveSubsystem.getRoll().getDegrees();
  }

  private double getPitch() {
    return mSwerveSubsystem.getPitch().getDegrees();
  }

  
}