package com.cyberknights4911.robot.control;

import java.util.function.DoubleSupplier;

/**
 * An abstraction layer for controller  stick bindings. 
 */
public interface StickBinding {

    /**
     * Given a desired stick action, return the actual stick input supplier.
     * 
     * @param action the stick action that is being bound.
     * @return the stick input supplier for the given action.
     */
    DoubleSupplier supplierFor(DriveStickAction action);
}
