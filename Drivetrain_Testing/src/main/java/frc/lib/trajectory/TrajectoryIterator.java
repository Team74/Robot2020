package frc.lib.trajectory;

import frc.lib.utils.geometry.*;

public class TrajectoryIterator<S extends State<S>> {
    protected final TrajectoryView<S> view;
    protected double progress = 0.0;
    protected TrajectorySamplePoint<S> currentSample;

    public TrajectoryIterator(final TrajectoryView<S> _view) {
        view = _view;

        //No effect if view is empty
        currentSample = view.sample(view.firstInterpolant());
        progress = view.firstInterpolant();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, view.lastInterpolant() - progress);
    }

    public TrajectorySamplePoint<S> getSample() {
        return currentSample;
    }

    public S getState() {
        return getSample().state();
    }

    public TrajectorySamplePoint<S> advance(double additionalProgress) {
        progress = Math.max(view.firstInterpolant(),
            (Math.min(view.lastInterpolant(), progress + additionalProgress)));
        currentSample = view.sample(progress);
        return currentSample;
    }

    public TrajectorySamplePoint<S> preview(double additionalProgress) {
        final double tProgress = Math.max(view.firstInterpolant(), 
            Math.min(view.lastInterpolant(), progress + additionalProgress));
        return view.sample(tProgress);
    }

    public Trajectory<S> trajectory() {
        return view.trajectory();
    }
}