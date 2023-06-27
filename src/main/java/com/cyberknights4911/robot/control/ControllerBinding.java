package com.cyberknights4911.robot.control;

import java.util.function.DoubleSupplier;

/**
 * An abstraction layer for controller button and stick bindings. 
 */
public interface ControllerBinding<E extends ButtonAction> {

    /**
     * Given a desired button action, return the actual button triggers.
     * 
     * @param action the button action that is being bound.
     * @return the input triggers that maps to the given action.
     */
    public Triggers triggersFor(E action);

    /**
     * Given a desired stick action, return the actual stick input supplier.
     * 
     * @param action the stick action that is being bound.
     * @return the stick input supplier for the given action.
     */
    public DoubleSupplier supplierFor(DriveStickAction action);
    
}
