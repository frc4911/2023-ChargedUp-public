package org.kingsschools.robot;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.cyberknights4911.robot.Robot;
import com.cyberknights4911.robot.RobotSupplier;

public class RobotSupplierTest {

    private RobotSupplier supplier;

    @BeforeEach
    public void setup() {
        supplier = new RobotSupplier();
    }

    @Test
    public void testNotNull() {
        assertNotNull(supplier.get());
    }

    @Test
    public void testCreatesCorrectRobotClass() {
        assertTrue(supplier.get() instanceof Robot);
    }

    @Test
    public void testCreatesNewInstanceEachTime() {
        assertNotSame(supplier.get(), supplier.get());
    }
    
}
