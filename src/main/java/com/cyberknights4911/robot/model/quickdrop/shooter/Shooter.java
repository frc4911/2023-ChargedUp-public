package com.cyberknights4911.robot.model.quickdrop.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    
    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Shooter", inputs);
    }

    public void setShooterSpeed(double speed) {
        shooterIO.setShooterSpeed(speed);
    }

    public void setHoodPosition(double percent) {
        shooterIO.setHoodPosition(percent);
    }
}
