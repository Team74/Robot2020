package frc.lib.utils.geometry;

import frc.lib.utils.Utilities;
import static frc.lib.utils.Utilities.kEpsilon;

public class Rotation2d implements IRotation2d<Rotation2d> {
    protected static final Rotation2d kIdentity = new Rotation2d();

    public static final Rotation2d identity() {
        return kIdentity;
    }

    protected final double cos_angle;
    protected final double sin_angle;
    protected double theta_degrees = 0;
    protected double theta_radians = 0;

    public Rotation2d() {
        this(1, 0, false);
    }

    public Rotation2d(double x, double y, boolean normalize) {
        if (normalize) {
            // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
            // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon) {
                sin_angle = y / magnitude;
                cos_angle = x / magnitude;
            } else {
                sin_angle = 0;
                cos_angle = 1;
            }
        } else {
            cos_angle = x;
            sin_angle = y;
        }
	    theta_degrees = Math.toDegrees(Math.atan2(sin_angle, cos_angle));
    }

    public Rotation2d(final Rotation2d other) {
        cos_angle = other.cos_angle;
        sin_angle = other.sin_angle;
        theta_degrees = Math.toDegrees(Math.atan2(sin_angle, cos_angle));
    }
    
    public Rotation2d(double theta_degrees){
    	cos_angle = Math.cos(Math.toRadians(theta_degrees));
    	sin_angle = Math.sin(Math.toRadians(theta_degrees));
    	this.theta_degrees = theta_degrees;
    }

    public Rotation2d(final Translation2d direction, boolean normalize) {
        this(direction.x(), direction.y(), normalize);
    }

    //Creates a Rotation2d object from a spedified radian amount
    public static Rotation2d fromRadians(double angle_radians) {
        return new Rotation2d(Math.cos(angle_radians), Math.sin(angle_radians), false);
    }

    //Creates a Rotation2d object from a specified degree amount
    public static Rotation2d fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public double cos() {
        return cos_angle;
    }

    public double sin() {
        return sin_angle;
    }

    public double tan() {
        if (Math.abs(cos_angle) < kEpsilon) {
            if (sin_angle >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sin_angle / cos_angle;
    }

    public double getRadians() {
        return Math.atan2(sin_angle, cos_angle);
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    public double getUnboundedDegrees() {
        return theta_degrees;
    }

    public Rotation2d rotateBy(final Rotation2d other) {
        return new Rotation2d(cos_angle * other.cos_angle - sin_angle * other.sin_angle,
        cos_angle * other.sin_angle + sin_angle * other.cos_angle, true);
    }

    public Rotation2d normal() {
        return new Rotation2d(-sin_angle, cos_angle, false);
    }

    //Inverse undoes this rotation
    public Rotation2d inverse() {
        return new Rotation2d(cos_angle, -sin_angle, false);
    }

    public boolean isParallel(final Rotation2d other) {
        return Utilities.epsilonEquals(Translation2d.cross(toTranslation(), other.toTranslation()), 0.0);
    }

    public Translation2d toTranslation() {
        return new Translation2d(cos_angle, sin_angle);
    }

    public Rotation2d nearestPole() {
        double pole_sin = 0.0;
        double pole_cos = 0.0;
        if (Math.abs(cos_angle) > Math.abs(sin_angle)) {
            pole_cos = Math.signum(cos_angle);
            pole_sin = 0.0;
        } else {
            pole_cos = 0.0;
            pole_sin = Math.signum(sin_angle);
        }
        return new Rotation2d(pole_cos, pole_sin, false);
    }

    @Override
    public Rotation2d interpolate(final Rotation2d other, double x) {
        if (x <= 0) {
            return new Rotation2d(this);
        } else if (x >= 1) {
            return new Rotation2d(other);
        }
        double angleDiff = inverse().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2d.fromRadians(angleDiff * x));
    }

    @Override
    public double distance(final Rotation2d other) {
        return inverse().rotateBy(other).getRadians();
    }

    @Override
    public boolean equals(Object other) {
        if (other == null || !(other instanceof Rotation2d)) return false;
        return distance((Rotation2d)other) < Utilities.kEpsilon;
    }

    @Override
    public Rotation2d getRotation() {
        return this;
    }
}