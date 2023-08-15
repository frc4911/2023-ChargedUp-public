package com.cyberknights4911.robot.control;

/**
 * An abstraction layer for controller button bindings. 
 */
public interface ButtonBinding<E extends ButtonAction> {

    /**
     * Given a desired button action, return the actual button triggers.
     * 
     * @param action the button action that is being bound.
     * @return the input triggers that maps to the given action.
     */
    Triggers triggersFor(E action);
    
}
