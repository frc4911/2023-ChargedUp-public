package com.cyberknights4911.robot.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An abstraction layer for controller button and stick bindings. 
 */
public interface ControllerBinding {

    /**
     * Given a desired button action, return the actual button trigger.
     * 
     * @param action the button action that is being bound.
     * @return the input trigger that maps to the given action.
     */
    public Trigger triggerFor(ButtonAction action);

    /**
     * Given a desired stick action, return the actual stick input supplier.
     * 
     * @param action the stick action that is being bound.
     * @return the stick input supplier for the given action.
     */
    public DoubleSupplier supplierFor(StickAction action);
    
}
