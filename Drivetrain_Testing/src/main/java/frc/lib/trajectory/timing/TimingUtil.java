package frc.lib.trajectory.timing;

import frc.lib.trajectory.*;
import frc.lib.utils.geometry.*;

import java.util.ArrayList;
import java.util.List;

//Basically a class to handle the time parameterization of trajectories
public class TimingUtil {
    public static <S extends State<S>> Trajectory<TimedState<S>> timeParameterizeTrajectory(
            boolean reverse,
            final DistanceView<S> distanceView,
            double stepSize,
            final List<TimingConstraints<S>> constraints,
            double startVelocity,
            double endVelocity,
            double maxVelocity,
            double maxAbsAcceleration) {
        final int numStates = (int) Math.ceil(distanceView.lastInterpolant() / stepSize + 1);
        List<S> states= new ArrayList<>(numStates);
        for (int i = 0; i < numStates; ++i) {
            states.add(distanceView.sample(Math.min(i * stepSize, distanceView.lastInterpolant())).state());
        }
        return timeParameterizeTrajectory(reverse, states, constraints, startVelocity, endVelocity, maxVelocity, maxAbsAcceleration);
    }
    
    public static <S extends State<S>> Trajectory<TimedState<S>> timeParameterizeTrajectory(
            boolean reverse,
            final List<S> states,
            final List<TimingConstraints<S>> constraints,
            double startVelocity,
            double endVelocity,
            double maxVelocity,
            double maxAbsAcceleration) {
        List<ConstrainedState<S>> constraintStates = new ArrayList<>(states.size());
        final double kEpsilon = 1e-6;

        // Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
        // parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
        // acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
        // there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
        // velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).
        ConstrainedState<S> predecessor = new ConstrainedState<>();
        predecessor.state = states.get(0);
        predecessor.distance = 0.0;
        predecessor.maxVelocity = startVelocity;
        predecessor.minAcceleration = -maxAbsAcceleration;
        predecessor.maxAcceleration = maxAbsAcceleration;
        for (int i = 0; i < states.size(); ++i) {
            //Add the new state
            constraintStates.add(new ConstrainedState<>());
            ConstrainedState<S> constraintState = constraintStates.get(i);
            constraintState.state = states.get(i);
            final double ds = constraintState.state.distance(predecessor.state);
            constraintState.distance = ds + predecessor.distance;
            
            // We may need to iterate to find the maximum end velocity and common acceleration, since acceleration
            // limits may be a function of velocity.
            while(true) {
                // Enforce global max velocity and max reachable velocity by global acceleration limit.
                // vf = sqrt(vi^2 + 2*a*d) PHYSICS!
                constraintState.maxVelocity = Math.min(maxVelocity, Math.sqrt(Math.pow(predecessor.maxVelocity, 2)) + 2.0 * predecessor.maxAcceleration * ds);
                if (Double.isNaN(constraintState.maxVelocity)) {
                    throw new RuntimeException();
                }
                // Enforce global max absolute acceleration
                constraintState.minAcceleration = -maxAbsAcceleration;
                constraintState.maxAcceleration = maxAbsAcceleration;

                // At this point, the state is full constructed, but no constraints have been applied aside from
                // predecessor
                // state max accel.

                // Enforce all velocity constraints.
                for (final TimingConstraints<S> constraint : constraints) {
                    constraintState.maxVelocity = Math.min(constraintState.maxVelocity, constraint.getMaxVelocity(constraintState.state));
                }
                if (constraintState.maxVelocity < 0.0) {
                    // This should never happen if constraints are well-behaved.
                    throw new RuntimeException();
                }

                // Now enforce all acceleration constraints.
                for (final TimingConstraints<S> constraint : constraints) {
                    final TimingConstraints.MinMaxAcceleration minMaxAcceleration = constraint.getMinMaxAcceleration(constraintState.state, (reverse ? -1.0 : 1.0) *constraintState.maxVelocity);
                    if (!minMaxAcceleration.valid()) {
                        // This should never happen if constraints are well-behaved.
                        throw new RuntimeException();
                    }
                    constraintState.minAcceleration = Math.max(constraintState.minAcceleration, reverse ? -minMaxAcceleration.maxAcceleration() : minMaxAcceleration.minAcceleration());
                    constraintState.maxAcceleration = Math.min(constraintState.maxAcceleration, reverse ? -minMaxAcceleration.minAcceleration() : minMaxAcceleration.maxAcceleration());
                }
                if (constraintState.minAcceleration > constraintState.maxAcceleration) {
                    // This should never happen if constraints are well-behaved.
                    throw new RuntimeException();
                }

                if (ds < kEpsilon) {
                    break;
                }
                // If the max acceleration for this constraint state is more conservative than what we had applied, we
                // need to reduce the max accel at the predecessor state and try again.
                // TODO: Simply using the new max acceleration is guaranteed to be valid, but may be too conservative.
                // Doing a search would be better. 
                // vf = sqrt(vi^2 + 2*a*d) Solved for a
                final double actualAcceleration = (Math.pow(constraintState.maxVelocity, 2) - Math.pow(predecessor.maxVelocity, 2)) / (2.0 * ds);
                if (constraintState.maxAcceleration < actualAcceleration - kEpsilon) {
                    predecessor.maxAcceleration = constraintState.maxAcceleration;
                } else {
                    if (actualAcceleration > predecessor.minAcceleration + kEpsilon) {
                        predecessor.maxAcceleration = actualAcceleration;
                    }
                    // If actual acceleration is less than predecessor min accel, we will repair during the backward
                    // pass.
                    break;                    
                }
            }
            predecessor = constraintState;
        }

        // Backward pass.
        ConstrainedState<S> successor = new ConstrainedState<>();
        successor.state = states.get(states.size() - 1);
        successor.distance = constraintStates.get(states.size() - 1).distance;
        successor.maxVelocity = endVelocity;
        successor.minAcceleration = -maxAbsAcceleration;
        successor.maxAcceleration = maxAbsAcceleration;
        for (int i = states.size() - 1; i >= 0; --i) {
            ConstrainedState<S> constraintState = constraintStates.get(i);
            final double ds = constraintState.distance - successor.distance; // Will be negative.

            while (true) {
                // Enforce reverse max reachable velocity limit.
                // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
                final double newMaxVelocity = Math.sqrt(Math.pow(successor.maxVelocity, 2) + 2.0 * successor.minAcceleration * ds);
                if (newMaxVelocity >= constraintState.maxVelocity) {
                    // No new limits to impose.
                    break;
                }
                constraintState.maxVelocity = newMaxVelocity;
                if (Double.isNaN(constraintState.maxVelocity)) {
                    throw new RuntimeException();
                }

                // Now check all acceleration constraints with the lower max velocity.
                for (final TimingConstraints<S> constraint : constraints) {
                    final TimingConstraints.MinMaxAcceleration minMaxAcceleration = constraint.getMinMaxAcceleration(constraintState.state, (reverse ? -1 : 1) * constraintState.maxVelocity);
                if (!minMaxAcceleration.valid()) {
                    throw new RuntimeException();
                }
                constraintState.minAcceleration = Math.max(constraintState.minAcceleration, reverse ? -minMaxAcceleration.maxAcceleration() : minMaxAcceleration.minAcceleration());
                constraintState.maxAcceleration = Math.min(constraintState.maxAcceleration, reverse ? -minMaxAcceleration.minAcceleration() : minMaxAcceleration.maxAcceleration());
                }
                if (constraintState.minAcceleration > constraintState.maxAcceleration) {
                    throw new RuntimeException();
                }

                if (ds > kEpsilon) {
                    break;
                }
                // If the min acceleration for this constraint state is more conservative than what we have applied, we
                // need to reduce the min accel and try again.
                // TODO: Simply using the new min acceleration is guaranteed to be valid, but may be too conservative.
                // Doing a search would be better.
                final double actualAcceleration = (Math.pow(constraintState.maxVelocity, 2) - Math.pow(successor.maxVelocity, 2)) / (2.0 * ds);
                if (constraintState.minAcceleration > actualAcceleration) {
                    successor.minAcceleration = constraintState.minAcceleration;
                } else {
                    successor.minAcceleration = actualAcceleration;
                    break;
                }
            }
            successor = constraintState;
        }
        // Integrate the constrained states forward in time to obtain the TimedStates.
        List<TimedState<S>> timedStates = new ArrayList<>(states.size());
        double t = 0.0;
        double s = 0.0;
        double v = 0.0;
        for (int i = 0; i < states.size(); ++i) {
            final ConstrainedState<S> constrainedState = constraintStates.get(i);
            // Advance t.
            final double ds = constrainedState.distance - s;
            final double acceleration = (Math.pow(constrainedState.maxVelocity, 2) - Math.pow(v, 2)) / (2.0 * ds);
            double dt = 0.0;
            if (i > 0) {
                timedStates.get(i- 1).setAcceleration(reverse ? -acceleration : acceleration);
                if (Math.abs(acceleration) > kEpsilon) {
                    dt = (constrainedState.maxVelocity - v) / acceleration;
                } else if ( Math.abs(v) > kEpsilon) {
                    dt = ds / v;
                } else {
                    throw new RuntimeException();
                }
            }
            t += dt;
            if (Double.isNaN(t) || Double.isInfinite(t)) {
                throw new RuntimeException();
            }

            v = constrainedState.maxVelocity;
            s = constrainedState.distance;
            timedStates.add(new TimedState<>(constrainedState.state, t, reverse ? -v : v, reverse ? -acceleration : acceleration));
        }
        return new Trajectory<>(timedStates);
    }

    protected static class ConstrainedState<S extends State<S>> {
        public S state;
        public double distance;
        public double maxVelocity;
        public double minAcceleration;
        public double maxAcceleration;
    }
}