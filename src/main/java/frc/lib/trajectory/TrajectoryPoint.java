package frc.lib.trajectory;

import frc.lib.utils.geometry.*;

public class TrajectoryPoint<S extends State<S>> {
    protected final S state;
    protected final int index;

    public TrajectoryPoint(final S _state, final int _index) {
        state = _state;
        index = _index;
    }

    public S state() {
        return state;
    }

    public int index() {
        return index;
    }
}