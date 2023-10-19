package com.cyberknights4911.robot.model.wham.arm;

import org.littletonrobotics.junction.Logger;
import com.cyberknights4911.robot.constants.DoublePreference;
import com.cyberknights4911.robot.model.wham.WhamConstants;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the arm.
 */
public final class ArmSubsystem extends SubsystemBase {
    public static final double SHOULDER_GEAR_RATIO = 120.0;
    public static final double WRIST_GEAR_RATIO = 60.0;
    public static final double TICKS_PER_REVOLUTION = 4096;
    public static final double DEGREES_PER_REVOLUTION = 360;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final TestMode testMode = new TestMode();

    private MoveArmCommand lastCommand = null;

    public ArmSubsystem(ArmIO armIO) {
        super();
        this.armIO = armIO;

        if (WhamConstants.Arm.IS_TUNING_ENABLED) {
            testMode.setupTestMode();
        }
    }

    public void setLastCommand(MoveArmCommand command) {
        lastCommand = command;
    }

    public boolean moveShoulder(double desiredShoulderPosition) {
        armIO.setShoulderPosition(convertDegreesToCtreTicks(desiredShoulderPosition));
        return true;
    }

    public boolean moveWrist(double desiredWristPosition) {
        armIO.setWristPosition(convertDegreesToCtreTicks(desiredWristPosition));
        return true;
    }

    public void resetCANCoders() {
        armIO.offsetShoulder();
        armIO.offsetWrist();
    }
    @Override
    public void periodic() {
        armIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);
    }

    public double getShoulderPositionDegrees() {
        return armIO.getShoulderEncoderDegrees();
    }

    public double getShoulderTrajectoryPosition() {
        return armIO.getShoulderTrajectoryPosition();
    }

    public double getWristTrajectoryPosition() {
        return armIO.getWristTrajectoryPosition();
    }

    public Command createArmCommand(SlurppSubsystem slurppSubsystem, ArmPositions desiredPosition) {
        return new MoveArmCommand(this, slurppSubsystem, desiredPosition);
    }

    public static double convertDegreesToCtreTicks(double degrees) {
        return degrees * TICKS_PER_REVOLUTION / DEGREES_PER_REVOLUTION;
    }

    public static double convertCtreTicksToDegrees(double ticks) {
        return ticks / TICKS_PER_REVOLUTION * DEGREES_PER_REVOLUTION;
    }

    /**
     * This is used to tune arm positions. Puts a tab on shuffleboard where arm positions can be adjusted.
     */
    private class TestMode {
        private void setupTestMode() {
            ShuffleboardTab tab = Shuffleboard.getTab("ARM TUNING");
            tab.add("SHOULDER+", increaseShoulder());
            tab.add("SHOULDER-", decreaseShoulder());
            tab.add("WRIST+", increaseWrist());
            tab.add("WRIST-", decreaseWrist());
        }

        CommandBase increaseShoulder() {
            return new ProxyCommand(
                () -> increase(lastCommand, lastCommand.getShoulderPosition()));
        }

        CommandBase decreaseShoulder() {
            return new ProxyCommand(
                () -> decrease(lastCommand, lastCommand.getShoulderPosition()));
        }

        CommandBase increaseWrist() {
            return new ProxyCommand(
                () -> increase(lastCommand, lastCommand.getWristPosition()));
        }

        CommandBase decreaseWrist() {
            return new ProxyCommand(
                () -> decrease(lastCommand, lastCommand.getWristPosition()));
        }

        private CommandBase increase(MoveArmCommand lastCommand, DoublePreference doublePreference) {
            return Commands.runOnce(() -> {
                doublePreference.setValue(doublePreference.getValue() + 1);
                lastCommand.rerunMoveToPosition();
            });
        }

        private CommandBase decrease(MoveArmCommand lastCommand, DoublePreference doublePreference) {
            return Commands.runOnce(() -> {
                doublePreference.setValue(doublePreference.getValue() - 1);
                lastCommand.rerunMoveToPosition();
            });
        }
    }
}
