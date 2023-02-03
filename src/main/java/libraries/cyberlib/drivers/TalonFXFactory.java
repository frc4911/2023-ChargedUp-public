package libraries.cyberlib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Factory for creating TalonFX objects. */
public final class TalonFXFactory {

    private TalonFXFactory() {}

    public static TalonFX createTalon(int id, String canivore) {
        if (canivore == null || canivore.isEmpty()) {
            return new LazyTalonFX(id);
        } else {
            return new LazyTalonFX(id, canivore);
        }
    }

    public static double getLastSet(TalonFX talonFX) {
        if (talonFX instanceof LazyTalonFX) {
            return ((LazyTalonFX) talonFX).lastSet;
        } else {
            return -1.0;
        }
    }

    /**
     * Wrapper around the TalonFX that reduces CAN bus / CPU overhead by skipping duplicate set
     * commands. (By default the Talon flushes the Tx buffer on every set call).
     */
    private static class LazyTalonFX extends TalonFX {
        protected double lastSet = Double.NaN;
        protected ControlMode lastControlMode = null;
    
        public LazyTalonFX(int deviceNumber) {
            super(deviceNumber);
        }
    
        public LazyTalonFX(int deviceNumber, String canbus) {
            super(deviceNumber, canbus);
        }
    
        @Override
        public void set(ControlMode mode, double value) {
            if (value != lastSet || mode != lastControlMode) {
                lastSet = value;
                lastControlMode = mode;
                super.set(mode, value);
            }
        }
    }
}
