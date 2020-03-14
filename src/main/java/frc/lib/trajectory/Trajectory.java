package frc.lib.trajectory;

import frc.lib.trajectory.TrajectoryPoint;

import frc.lib.utils.geometry.*;

import java.util.List;
import java.util.ArrayList;

public class Trajectory<S extends State<S>> {
    protected final List<TrajectoryPoint<S>> points;

    protected double defaultVelocity = 0.0;

    //Empty trajectory
    public Trajectory() {
        points = new ArrayList<TrajectoryPoint<S>>();
    }
    //Trajectory with List of object S
    public Trajectory(final List<S> states) {
        points =  new ArrayList<TrajectoryPoint<S>>();
        for (int i = 0; i < states.size(); ++i) {
            points.add(new TrajectoryPoint<>(states.get(i), i));
        }
    }

    public void setDefaultVelocity(double _defaultVelocity) {
        this.defaultVelocity = _defaultVelocity;
    }

    public double defaultVelocity() {
        return defaultVelocity;
    }

    public boolean isEmpty() {
        return points.isEmpty();
    }

    public int length() {
        return points.size();
    }

    public TrajectoryPoint<S> getPoint(final int index) {
        return points.get(index);
    }

    public S getState(final int index) {
        return points.get(index).state();
    }

    public S getFirstEntry() {
        return points.get(0).state();
    }

    public S getLastEntry() {
        return points.get(points.size() - 1).state();
    }
}