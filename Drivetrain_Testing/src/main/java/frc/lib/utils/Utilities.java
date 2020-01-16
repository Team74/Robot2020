package frc.lib.utils;

import frc.lib.utils.geometry.*;
public class Utilities {
    
    //Really small number
    public static final double kEpsilon = 1e-12;

    //Prevent this class from being instantiated
    private Utilities(){

    }

    public static double handleDeadband1D(double input, double kDeadband) {
        return Math.abs(input) <= kDeadband ? 0 : input; 
    }

    public static Translation2d handleDeadband2D(Translation2d input, double kDeadband) {
        return input.norm() >= kDeadband ? new Translation2d() : input;
    }

    public static boolean doubleToBool(double input){
        boolean temp = Math.abs(input) > 0.0 ? true : false;
        return temp;
    }

    public static double encoderToAngle(double encoder, double countsPerRev){
        double angle = encoder/countsPerRev * 360;
        return angle;
    }

    public static double angleToEncoder(double angle, double countsPerRev){
        double encoder = (angle/360) * countsPerRev;
        return encoder;
    }

    public static Translation2d angleToUnitVector(double _angle){
        return Translation2d.fromPolar(new Rotation2d(_angle), 1.0);
    } 

    /**
     * Function to place any angle on the appropriate -180 to 180 range 
     */
    public static double boundAngleNeg180to180Degrees(double angle){
        while(angle >= 180.0) {angle -= 360.0;}
        while(angle < -180.0) {angle += 360.0;}
        return angle;
    }

    /**
     * Function to place any angle on the appropriate 0-360 range, ie. 540, becomes 180
     * @param _angleDegrees angle to bound in degrees
     * @return Bounded angle
     */
    public static double boundAngle0to360Degrees(double _angleDegrees) {
        while (_angleDegrees >= 360.0) { _angleDegrees -= 360.0; }
        while (_angleDegrees < 0.0) { _angleDegrees += 360.0; }
        return _angleDegrees;
    }

    /**
     * Function to place any angle on the appropriate 0-360 range, ie. 540, becomes 180
     * @param _angleRadians angle to bound in degrees
     * @return Bounded angle
     */
    public static double boundAngle0to2PiRadians(double _angleRadians) {
        return Math.toRadians(boundAngle0to360Degrees(Math.toDegrees(_angleRadians)));
    }

    /**
     * Takes a scope and a target and places the target in the same 0-360 range as the scope. 
     * For example reference of 180 and target of 370 would be 10 and the reference of 450 and target of 180 would be 540.
     * @param scopeReference 0-360 range to place target in
     * @param newAngle Target to scope
     * @return Angle in scope
     */
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle){
    	double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if(lowerOffset >= 0){
        	lowerBound = scopeReference - lowerOffset;
        	upperBound = scopeReference + (360 - lowerOffset);
        }else{
        	upperBound = scopeReference - lowerOffset; 
        	lowerBound = scopeReference - (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
        	newAngle += 360; 
        }
        while(newAngle > upperBound){
        	newAngle -= 360; 
        }
        if(newAngle - scopeReference > 180){
        	newAngle -= 360;
        }else if(newAngle - scopeReference < -180){
        	newAngle += 360;
        }
        return newAngle;
    }

    /**
     * Same as the above function but with a defined scope
     * @param scopeFloor
     * @param scopeCeiling
     * @param argument
     * @return Bounded argument
     */
    public static double boundToScope(double scopeFloor, double scopeCeiling, double argument){
    	double stepSize = scopeCeiling - scopeFloor;
    	while(argument >= scopeCeiling) {argument -= stepSize;}
    	while(argument < scopeFloor) {argument += stepSize;}
    	return argument;
    }
    
    //Functions to limit the input to a range a values
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    //Takes to end points and a percent between them and returns the point that is a percent between them
    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b-a) * x;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return ((a - epsilon <= b) && (a + epsilon >= b));
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}