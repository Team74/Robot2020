package frc.lib.trajectory;

import frc.lib.trajectory.TrajectoryPoint;

import frc.lib.utils.geometry.*;

public class TrajectorySamplePoint<S extends State<S>> {
    protected final S state;
    protected final int indexFloor;
    protected final int indexCeiling;

    public TrajectorySamplePoint(final S _state, int _indexFloor, int _indexCeiling) {
        state = _state;
        indexFloor = _indexFloor;
        indexCeiling = _indexCeiling;
    }

    public TrajectorySamplePoint(final TrajectoryPoint<S> _point) {
        state = _point.state();
        indexFloor = indexCeiling = _point.index();
    }

    public S state() {
        return state;
    }

    public int indexFloor() {
        return indexFloor;
    }

    public int indexCeiling() {
        return indexCeiling;
    }
}