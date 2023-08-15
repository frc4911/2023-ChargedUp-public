package libraries.cyberlib.drivers;

import com.ctre.phoenix.sensors.BasePigeon;

/** Factory for creating BasePigeon objects. */
public interface PigeonFactory {
    BasePigeon create(int id);
}
