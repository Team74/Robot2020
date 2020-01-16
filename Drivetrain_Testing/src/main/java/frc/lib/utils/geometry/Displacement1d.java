package frc.lib.utils.geometry;

import frc.lib.utils.Utilities;

public class Displacement1d implements State<Displacement1d> {

    protected final double displacement;

    public Displacement1d() {
        displacement = 0.0;
    }

    public Displacement1d(double _displacement) {
        displacement = _displacement;
    }

    public double x() {
        return displacement;
    }

    @Override
    public Displacement1d interpolate(final Displacement1d other, double i) {
        return new Displacement1d(Utilities.interpolate(displacement, other.displacement, i));
    }

    @Override
    public double distance(final Displacement1d other) {
        return Math.abs(x() - other.x());
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Displacement1d)) return false;
        return Utilities.epsilonEquals(x(), ((Displacement1d)other).x());
    }
}