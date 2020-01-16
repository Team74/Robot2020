package frc.lib.trajectory.timing;

import frc.lib.trajectory.timing.TimingConstraints;

import frc.lib.utils.geometry.Pose2dWithCurvature;

public class CentripitalAccelerationConstraint implements TimingConstraints<Pose2dWithCurvature> {
    final double kMaxCentripetalAcceleration;

    public CentripitalAccelerationConstraint(final double _maxCentripetalAcceleration) {
        kMaxCentripetalAcceleration = _maxCentripetalAcceleration;
    }

    public double getMaxVelocity(final Pose2dWithCurvature entry) {
        return Math.sqrt(Math.abs(kMaxCentripetalAcceleration / entry.getCurvature()));
    }

    public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithCurvature entry, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }

}