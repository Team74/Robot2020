package frc.lib.trajectory;

import frc.lib.utils.geometry.*;
import frc.lib.utils.Utilities;

public class DistanceView<S extends State<S>> implements TrajectoryView<S> {
    protected final Trajectory<S> trajectory;
    protected final double[] distances;

    public DistanceView(final Trajectory<S> _trajectory) {
        trajectory = _trajectory;
        distances = new double[trajectory.length()];
        distances[0] = 0.0;
        for (int i = 1; i < trajectory.length(); ++i) {
            distances[i] = distances[i - 1] + trajectory.getState(i - 1).distance(trajectory.getState(i));
        }
    }

    @Override
    public TrajectorySamplePoint<S> sample(double distance) {
        if (distance >= lastInterpolant())
            return new TrajectorySamplePoint<S>(trajectory.getPoint(trajectory.length() - 1));
        if (distance <= 0.0)
            return new TrajectorySamplePoint<S>(trajectory.getPoint(0));
        for (int i = 1; i < distances.length; ++i) {
            final TrajectoryPoint<S> s = trajectory.getPoint(i);
            if (distances[i] >= distance) {
                final TrajectoryPoint<S> prevS = trajectory.getPoint(i -1);
                if (Utilities.epsilonEquals(distances[i], distances[i - 1])) {
                    return new TrajectorySamplePoint<S>(s);
                } else {
                    return new TrajectorySamplePoint<S>(prevS.state().interpolate(s.state(), (distance - distances[i - 1]) / (distances[i] - distances[i - 1])), i - 1, i);
                }
            }
        }
        throw new RuntimeException();
    }

    @Override
    public double lastInterpolant() {
        return distances[distances.length - 1];
    }
    
    @Override
    public double firstInterpolant() {
        return 0.0;
    }

    @Override
    public Trajectory<S> trajectory() {
        return trajectory;
    }
}