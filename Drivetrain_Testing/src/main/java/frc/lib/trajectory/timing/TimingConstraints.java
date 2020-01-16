package frc.lib.trajectory.timing;

public interface TimingConstraints<S> {
    double getMaxVelocity(S entry);

    MinMaxAcceleration getMinMaxAcceleration(S entry, double velocity);

    public static class MinMaxAcceleration {
        protected final double minAcceleration;
        protected final double maxAcceleration;

        public static MinMaxAcceleration kNoLimits = new MinMaxAcceleration();

        //No limits
        public MinMaxAcceleration() {
            minAcceleration = Double.NEGATIVE_INFINITY;
            maxAcceleration = Double.POSITIVE_INFINITY;
        }

        public MinMaxAcceleration(double _minAcceleration, double _maxAcceleration){
            minAcceleration = _minAcceleration;
            maxAcceleration = _maxAcceleration;
        }

        public double minAcceleration() {
            return minAcceleration;
        }

        public double maxAcceleration() {
            return maxAcceleration;
        }

        public boolean valid() {
            return minAcceleration() <= maxAcceleration();
        }
    }
}