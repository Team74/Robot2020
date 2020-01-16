package frc.lib.utils.geometry;

import org.opencv.core.Mat;

import frc.lib.utils.Utilities;
//Stores x, y positions as vectors, or transformations from the origin

public class Translation2d implements ITranslation2d<Translation2d> {
    protected static final Translation2d kIdentity = new Translation2d();

    public static final Translation2d identity() {
        return kIdentity;
    }

    protected double x;
    protected double y;
    
    public Translation2d(){
        x = 0.0;
        y = 0.0;
    }

    public Translation2d(double _x, double _y){
        x = _x;
        y = _y;
    }

    public Translation2d(final Translation2d start, final Translation2d end) {
        x = end.x - start.x;
        y = end.y - start.y;
    }

    public Translation2d(final Translation2d other) {
        x = other.x;
        y = other.y;
    }

    public static Translation2d fromXY(double _x, double _y) {
        return fromXYS(_x, _y, 1.0);
    }

    public static Translation2d fromXYS(double _x, double _y, double _s) {
        return new Translation2d(_x , _y).scale(_s);
    }

    public static Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.cos(), direction.sin()).scale(magnitude);
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public void setX(double _x){
        x = _x;
    }

    public void setY(double _y){
        y = _y;
    }
    /**
     * You can combine transforms by adding the x and y shifts AKA adding vectors
     * @param other
     * @return This + other
     */
    public Translation2d translateBy(final Translation2d other) {
        return new Translation2d(x + other.x, y + other.y);
    }

    //You can rotate Translation2d's https://en.wikipedia.org/wiki/Rotation_matrix
    public Translation2d rotateBy(final Rotation2d _rotation) {
        return new Translation2d(x * _rotation.cos() - y * _rotation.sin(), x * _rotation.sin() + y * _rotation.cos());
    }

    public Rotation2d direction() {
        return new Rotation2d(x, y, true);
    }
    
    //Inverse returns a Translations2d that undoes the Translation2d it is called from, hence -x, -y
    public Translation2d inverse() {
        return new Translation2d(-x, -y);
    }

    @Override
    public Translation2d interpolate(final Translation2d other, double i) {
        if (i <= 0.0) {
            return new Translation2d(this);
        } else if (i >= 1.0) {
            return new Translation2d(other);
        }
        return extrapolate(other, i);
    }

    public Translation2d extrapolate(final Translation2d other, double e) {
        return new Translation2d(e * (other.x - x) + x, e * (other.y - y) + y);
    }

    public Translation2d scale(double _s) {
        return new Translation2d(x * _s, y * _s);
    }

    //The "norm" of a transform is th Euclidean distance in x and y. AKA the length of the vector 
    public double norm() {
        return Math.hypot(x, y);
    }

    //Returns norm squared
    public double norm2() {
        return Math.pow(x, 2) + Math.pow(y, 2);
    }

    public boolean epsilonEquals(final Translation2d other, double epsilon) {
        return Utilities.epsilonEquals(x(), other.x(), epsilon) && Utilities.epsilonEquals(y(), other.y(), epsilon);
    }

    //Implementation of the dot product for two points or "vector points"
    public static double dot(Translation2d a, Translation2d b) {
        return a.x * b.x + a.y * b.y;
    }

    //Implementation of the cross product for two points or "vector points"
    public static double cross(final Translation2d a, final Translation2d b) {
        return a.x * b.y - a.y * b.x;
    }

    //Finds the distance of a straight line between the point of the Translation2d this is called from and other
    @Override
    public double distance(final Translation2d other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Translation2d)) return false;
        return distance((Translation2d)other) < Utilities.kEpsilon;
    }

    @Override
    public Translation2d getTranslation() {
        return this;
    }
}