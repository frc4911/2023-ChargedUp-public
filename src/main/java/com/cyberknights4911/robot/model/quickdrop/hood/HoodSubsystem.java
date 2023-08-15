package com.cyberknights4911.robot.model.quickdrop.hood;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private final HoodIO hoodIO;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public enum HoodPositions {
        STOWED(0),
        H1(10000),
        H2(20000);

        double position;

        HoodPositions(double position) {
            this.position = position;
        }

        public double get() {
            return position;
        }
    }

    private HoodPositions desiredHoodPosition = HoodPositions.STOWED;

    public HoodSubsystem(HoodIO hoodIO) {
        this.hoodIO = hoodIO;
    }

    public void setDesiredHoodPosition(HoodPositions desiredPosition) {
        desiredHoodPosition = desiredPosition;
        moveHood();
    }

    public void moveHood() {
        hoodIO.setPosition(desiredHoodPosition.get());
    }

    @Override
    public void periodic() {
        hoodIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Hood", inputs);
    }
}
