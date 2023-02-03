package libraries.cyberlib.utils;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a holonomic drive signal.
 */
public class HolonomicDriveSignal {
    private final Translation2d translation;
    private final double rotation;
    private final boolean fieldOriented;

    public HolonomicDriveSignal(Translation2d translation2d, double rotation, boolean fieldOriented) {
        this.translation = translation2d;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }

    @Override
    public String toString() {
        return String.format("HolonomicDriveSignal T%s, R(%.3f), FieldOriented(%b)", translation.toString(), rotation,
                fieldOriented);
    }
}
