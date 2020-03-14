package frc.lib.trajectory;

import frc.lib.spline.*;
import frc.lib.trajectory.timing.*;
import frc.lib.utils.geometry.*;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtil {
    public static <S extends IPose2d<S>> Trajectory<S> mirror(final Trajectory<S> _trajectory) {
        List<S> waypoints = new ArrayList<>(_trajectory.length());
        for (int i = 0; i < _trajectory.length(); ++i) {
            waypoints.add(_trajectory.getState(i).mirror());
        }
        return new Trajectory<>(waypoints);
    }

    public static  <S extends IPose2d<S>> Trajectory<TimedState<S>> mirrorTimed(final Trajectory<TimedState<S>>  _trajectory, double _defaultVelocity) {
        List<TimedState<S>> waypoints = new ArrayList<>(_trajectory.length());
        for (int i = 0; i < _trajectory.length(); ++i) {
            TimedState<S> timedState = _trajectory.getState(i);
            waypoints.add(new TimedState<S>(timedState.state().mirror(), timedState.t(), timedState.velocity(), timedState.acceleration()));
        }
        Trajectory<TimedState<S>> tempTrajectory = new Trajectory<TimedState<S>>(waypoints);
        tempTrajectory.setDefaultVelocity(_defaultVelocity);
        return tempTrajectory;
    }

    public static <S extends IPose2d<S>> Trajectory<S> transform(final Trajectory<S> _trajectory, Pose2d _transform) {
        List<S> waypoints = new ArrayList<>(_trajectory.length());
        for (int i = 0; i < _trajectory.length(); ++i) {
            waypoints.add(_trajectory.getState(i).transformBy(_transform));
        }
        return new Trajectory<>(waypoints);
    }

    public static Trajectory<Pose2dWithCurvature> trajectoryFromSplineWaypoints(final List<Pose2d> waypoints, double maxDx, double maxDy, double maxDTheta) {
        List<QuinticHermiteSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermiteSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        QuinticHermiteSpline.optimizeSpline(splines);
        return trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
    }

    public static Trajectory<Pose2dWithCurvature> trajectoryFromSplines(final List<? extends Spline> splines, double maxDx, double maxDy, double maxDTheta) {
        return new Trajectory<>(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }
}