package frc.lib.utils.geometry;

import frc.lib.utils.geometry.Translation2d;
import frc.lib.utils.geometry.Rotation2d;
import frc.lib.utils.Utilities;
import frc.lib.utils.geometry.IPose2d;

public class Pose2d implements IPose2d<Pose2d> {

    protected static final Pose2d kIdentity = new Pose2d();

    public static final Pose2d identity( ){
        return kIdentity;
    }

    private final static double kEps = 1E-9;

    protected final Translation2d translation;
    protected final Rotation2d rotation;

    public Pose2d() {
        translation = new Translation2d();
        rotation = new Rotation2d();
    }

    public Pose2d(double x, double y, double degrees) {
        translation = new Translation2d(x, y);
        rotation = Rotation2d.fromDegrees(degrees);
    }

    public Pose2d(double x, double y, final Rotation2d _rotation) {
        translation = new Translation2d(x, y);
        rotation = _rotation;
    }

    public Pose2d(final Translation2d _translation, final Rotation2d _rotation) {
        translation = _translation;
        rotation = _rotation;
    }

    public Pose2d(final Pose2d other) {
        translation = new Translation2d(other.translation);
        rotation = new Rotation2d(other.rotation);
    }

    public static Pose2d fromTranslation(final Translation2d _translation) {
        return new Pose2d(_translation, new Rotation2d());
    }

    public static Pose2d fromRotation(final Rotation2d _rotation) {
        return new Pose2d(new Translation2d(), _rotation);
    }

    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(translation.translateBy(other.translation.rotateBy(rotation)),
        rotation.rotateBy(other.rotation));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public Pose2d inverse() {
        Rotation2d rotation_inverted = rotation.inverse();
        return new Pose2d(translation.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    public Pose2d normal() {
        return new Pose2d(translation, rotation.normal());
    }

    /**
     * Finds the point where the heading of this pose intersects the heading of another. Returns (+INF, +INF) if
     * parallel.
     */

    public Translation2d intersection(final Pose2d _other) {
        final Rotation2d otherRotation = _other.getRotation();
        if (rotation.isParallel(otherRotation)) {
            //Lines are parallel.
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(rotation.cos()) < Math.abs(otherRotation.cos())) {
            return intersectionInternal(this, _other);
        } else {
            return intersectionInternal(_other, this);
        }
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isColinear(final Pose2d _other) {
        if (!getRotation().isParallel(_other.getRotation())) return false;
        final Twist2d twist = log(inverse().transformBy(_other));
        return (Utilities.epsilonEquals(twist.dy, 0.0) && Utilities.epsilonEquals(twist.dtheta, 0.0));
    }

    private static Translation2d intersectionInternal(final Pose2d _a, final Pose2d _b) {
        final Rotation2d aR = _a.getRotation();
        final Rotation2d bR = _b.getRotation();
        final Translation2d aT = _a.getTranslation();
        final Translation2d bT = _b.getTranslation();

        final double tanB = bR.tan();
        final double t = ((aT.x() - bT.x()) * tanB + bT.y() - aT.y()) / (aR.sin() - aR.cos() * tanB);
        if (Double.isNaN(t)) {
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return aT.translateBy(aR.toTranslation().scale(t));
    }

    /*
        Obtain a new Pose2d from a (constant curvature) velocity. See:
        https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
    */
    public static Pose2d exp(final Twist2d delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < kEps) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation2d(cos_theta, sin_theta, false));
    }

    /*
        Logical inverse of the above.
    */
    public static Twist2d log(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().cos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new Twist2d(translation_part.x(), translation_part.y(), dtheta);
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    public boolean epsilonEquals(final Pose2d other, double epsilon) {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon) && getRotation().isParallel(other.getRotation());
    }

    /*
        Do Twist inerpolation of this pose assuming constant curvature
    */
    @Override
    public Pose2d interpolate(final Pose2d other, double x) {
        if (x <= 0) {
            return new Pose2d(this);
        } else if (x >= 1) {
            return new Pose2d(other); 
        }
        final Twist2d twist = Pose2d.log(inverse().transformBy(other));
        return transformBy(Pose2d.exp(twist.scaled(x)));
    }

    @Override
    public double distance(final Pose2d other) {
        return Pose2d.log(inverse().transformBy(other)).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Pose2d)) return false;
        return epsilonEquals((Pose2d)other, Utilities.kEpsilon);
    }
    
    @Override    
    public Pose2d getPose() {
        return this;
    }

    @Override
    public Pose2d mirror() {
        return new Pose2d(new Translation2d(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
    }
}