package com.cyberknights4911.robot.model.quickdrop;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class LogitechController extends CommandGenericHID {
    
  public LogitechController(int port) {
        super(port);
    }

    /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getRawAxis(Axis.kLeftX.value);
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return getRawAxis(Axis.kRightX.value);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getRawAxis(Axis.kLeftY.value);
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return getRawAxis(Axis.kRightY.value);
  }

  public Trigger leftBumper() {
    return button(Button.LeftBumper.value);
  }

  public Trigger rightBumper() {
    return button(Button.RightBumper.value);
  }
  
  public enum Axis {
    kLeftX(0),
    kRightX(4),
    kLeftY(1),
    kRightY(5);

    public final int value;

    Axis(int value) {
      this.value = value;
    }
  }
  
  public enum Button {
    LeftBumper(5),
    RightBumper(6),
    LeftStick(9),
    RightStick(10),
    A(1),
    B(2),
    X(3),
    Y(4),
    Back(7),
    Start(8);

    public final int value;

    Button(int value) {
      this.value = value;
    }
  }
}

