package frc.lib.trajectory;

import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.TrajectoryPoint;
import frc.lib.trajectory.TrajectorySamplePoint;

import frc.lib.utils.geometry.*;
import frc.lib.utils.Utilities;

public class TimedView<S extends State<S>> implements TrajectoryView<TimedState<S>> {
    protected final Trajectory<TimedState<S>> trajectory;
    protected final double startTime;
    protected final double endTime;

    public TimedView(Trajectory<TimedState<S>> _trajectory) {
        trajectory = _trajectory;
        startTime = trajectory.getState(0).t();
        endTime = trajectory.getState(trajectory.length() - 1).t();
    }

    @Override
    public double firstInterpolant() {
        return startTime;
    }

    @Override
    public double lastInterpolant() {
        return endTime;
    }

    @Override
    public TrajectorySamplePoint<TimedState<S>> sample(double t) {
        if (t >= endTime) {
            return new TrajectorySamplePoint<>(trajectory.getPoint(trajectory.length() - 1));
        }
        if (t <= startTime) {
            return new TrajectorySamplePoint<>(trajectory.getPoint(0));
        }
        for (int i = 1; i < trajectory.length(); ++i) {
            final TrajectoryPoint<TimedState<S>> s = trajectory.getPoint(i - 1);
            if (s.state().t() >= t) {
                final TrajectoryPoint<TimedState<S>> prevS = trajectory.getPoint(i-1);
                if (Utilities.epsilonEquals(s.state().t(), prevS.state().t())) {
                    return new TrajectorySamplePoint<>(s);
                }
                return new TrajectorySamplePoint<TimedState<S>>(prevS.state().interpolate(s.state(), (t - prevS.state().t()) / (s.state().t() - prevS.state().t())), i - 1, i);
            }
        }
        throw new RuntimeException();
    }

    @Override
    public Trajectory<TimedState<S>> trajectory() {
        return trajectory;
    }
}