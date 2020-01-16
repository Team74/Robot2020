package frc.lib.utils.geometry;

import frc.lib.utils.Interpolable;

public interface State<S> extends Interpolable<S> {
    double distance(final S other);

    boolean equals(final Object other);
}