package libraries.cyberlib.utils;

public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

}
