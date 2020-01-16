package frc.lib.utils.geometry;

import frc.lib.utils.Utilities;

public class Pose2dWithCurvature implements IPose2d<Pose2dWithCurvature>, ICurvature<Pose2dWithCurvature> {
    protected static final Pose2dWithCurvature kIdentity = new Pose2dWithCurvature();

    public static final Pose2dWithCurvature identity() {
        return kIdentity;
    }

    protected final Pose2d pose;
    protected final double curvature;
    protected final double dcurvature_ds;

    public Pose2dWithCurvature() {
        pose = new Pose2d();
        curvature = 0.0;
        dcurvature_ds = 0.0;
    }

    public Pose2dWithCurvature(final Pose2d _pose, double _curvature) {
        pose = _pose;
        curvature = _curvature;
        dcurvature_ds = 0.0;
    }

    public Pose2dWithCurvature(final Pose2d _pose, double _curvature, double _dcurvature_ds) {
        pose = _pose;
        curvature = _curvature;
        dcurvature_ds = _dcurvature_ds;
    }

    public Pose2dWithCurvature(final Translation2d _translation, final Rotation2d _rotation, double _curvature) {
        pose = new Pose2d(_translation, _rotation);
        curvature = _curvature;
        dcurvature_ds = 0.0;
    }

    public Pose2dWithCurvature(final Translation2d _translation, final Rotation2d _rotation, double _curvature, double _dcurvature_ds) {
        pose = new Pose2d(_translation, _rotation);
        curvature = _curvature;
        dcurvature_ds = _dcurvature_ds;
    }
    
    @Override
    public final Pose2d getPose() {
        return pose;
    }
    
    @Override
    public double getCurvature() {
        return curvature;
    }
    
    @Override
    public double getDCurvatureDs() {
        return dcurvature_ds;
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Pose2dWithCurvature)) return false;
        Pose2dWithCurvature p2dwc = (Pose2dWithCurvature) other;
        return getPose().equals(p2dwc.getPose()) &&
            Utilities.epsilonEquals(getCurvature(), p2dwc.getCurvature()) &&
            Utilities.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }
    
    @Override
    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    @Override
    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }
    
    @Override
    public double distance(final Pose2dWithCurvature other) {
        return getPose().distance(other.getPose());
    }
    
    @Override
    public Pose2dWithCurvature transformBy(Pose2d _transform) {
        return new Pose2dWithCurvature(getPose().transformBy(_transform), getCurvature(), getDCurvatureDs());
    }
    
    @Override
    public Pose2dWithCurvature mirror() {
        return new Pose2dWithCurvature(getPose().mirror().getPose(), -getCurvature(), -getDCurvatureDs());
    }
    
    @Override
    public Pose2dWithCurvature interpolate(final Pose2dWithCurvature other, double x) {
        return new Pose2dWithCurvature(getPose().interpolate(other.getPose(), x),
         Utilities.interpolate(getCurvature(), other.getCurvature(), x), 
         Utilities.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

}