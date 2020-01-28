package frc.lib.utils;

public class Units {
    public static double rpmToRadsPerSec(double _rpm) {
        return (_rpm * 2 * Math.PI) / 60.0;
    }

    public static double radsPerSecToRPM(double _radPerSec) {
        return (_radPerSec * 60) / (2 * Math.PI);
    }

    public static double inchesToMeters(double _inches) {
        return _inches * 0.0254;
    }

    public static double metersToInches(double _meters) {
        return _meters / 0.0254;
    }

    public static double feetToMeters(double _feet) {
        return inchesToMeters(_feet * 12);
    }

    public static double metersToFeet(double _meters) {
        return metersToInches(_meters) / 12;
    }

    public static double degreesToRadians(double _degrees) {
        return Math.toRadians(_degrees);
    }

    public static double radiansToDegrees(double _radians) {
        return Math.toDegrees(_radians);
    }
}