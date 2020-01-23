package frc.robot;

import frc.lib.utils.Units;
import frc.lib.utils.geometry.*;
import frc.lib.trajectory.*;
import frc.lib.trajectory.timing.*;

import java.util.List;

import java.util.ArrayList;

/**
 * Class to be called to command the drivetrain
 */
public class DrivePlanner {
    public static DrivePlanner instance;
    public static DrivePlanner getInstance() {
        if (instance == null) {
            instance = new DrivePlanner();
        }
        return instance;
    }

    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = .25;
    private static final double kMaxDTheta = Math.toRadians(5.0);

    private TrajectoryIterator<TimedState<Pose2dWithCurvature>> currentTrajectory;
    private boolean isReversed = false;
    private double lastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> setpoint = new TimedState<>(Pose2dWithCurvature.identity());
    private Pose2d error = Pose2d.identity();
    private Command command = new Command();

    private double dt = 0.0;


    private DrivePlanner() {

    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> _trajectory) {
        currentTrajectory = _trajectory;
    }

    /**
     * Generate a trajectory with starting and ending velocity 0;
     * @param _revearsed
     * @param _waypoints
     * @param _constraints
     * @param _maxVelocity
     * @param _maxAcceleration
     * @param _maxVoltage
     * @return
     */
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
        boolean _revearsed, 
        final List<Pose2d> _waypoints, 
        final List<TimingConstraints<Pose2dWithCurvature>> _constraints,
        double _maxVelocity, // inches/s
        double _maxAcceleration, // inches/s^2
        double _maxVoltage) {
    return generateTrajectory(
        _revearsed, 
        _waypoints, 
        _constraints, 
        0.0, 
        0.0, 
        _maxVelocity, 
        _maxAcceleration,
        _maxVoltage);
    }

    /**
     * Can generate trajectories with non 0 starting and ending velocities, also handles all trajectory inital parameterization
     * @param _reversed
     * @param _waypoints
     * @param _constraints
     * @param _startVelocity
     * @param _endVelocity
     * @param _maxVelocity
     * @param _maxAcceleration
     * @param _maxVoltage
     * @return
     */
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean _reversed,
            final List<Pose2d> _waypoints,
            final List<TimingConstraints<Pose2dWithCurvature>> _constraints,
            double _startVelocity,
            double _endVelocity,
            double _maxVelocity, // inches/s^2
            double _maxAcceleration, // inches/s^2
            double _maxVoltage) {
        List<Pose2d> waypointsMaybeFlipped = _waypoints;
        //TODO: Make sure the flip constant is consistent with the way we define our cordinate system
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(0, -1, false));
        if (_reversed) {
            waypointsMaybeFlipped = new ArrayList<>(_waypoints.size());
            for (int i = 0; i < _waypoints.size(); ++i) {
                waypointsMaybeFlipped.add(_waypoints.get(i).transformBy(flip));
            }
        }

        //Create a trajectory from the spline waypoints
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(waypointsMaybeFlipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (_reversed) {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                Pose2dWithCurvature ithState = trajectory.getState(i);
                flipped.add(new Pose2dWithCurvature(ithState.getPose().transformBy(flip), -ithState.getCurvature(), ithState.getDCurvatureDs()));
            }

            trajectory = new Trajectory<>(flipped);
        }

        List<TimingConstraints<Pose2dWithCurvature>> allConstraints = new ArrayList<>();

        if (_constraints != null) {
            allConstraints.addAll(_constraints);
        }

        //Generate the timed trajectory
        Trajectory<TimedState<Pose2dWithCurvature>> timedTrajectory = TimingUtil.timeParameterizeTrajectory(_reversed, new DistanceView<>(trajectory), kMaxDx, allConstraints,
                _startVelocity, _endVelocity, _maxVelocity, _maxAcceleration);
        return timedTrajectory;
    }

    public Command followTrajectory(double _timeStamp, Pose2d _currentPose) {
        if (currentTrajectory == null) return new Command();

        if (currentTrajectory.getProgress() == 0.0 && !Double.isFinite(lastTime)) lastTime = _timeStamp;

        dt = _timeStamp - lastTime;

        if (!currentTrajectory.isDone()) {
            //Generate Feedfoward voltages
            final double velocity_m = Units.inchesToMeters(setpoint.velocity());
            final double acceleration_m = Units.inchesToMeters(setpoint.acceleration());
            final double curvature = Units.metersToInches(setpoint().state().getCurvature());
            final double dcurvatureDs = Units.metersToInches(Units.metersToInches(setpoint.state().getDCurvatureDs()));
        }
        return new Command();
    }

    public boolean isDone() {
        return currentTrajectory != null && currentTrajectory.isDone();
    }

    public Pose2d error() {
        return error;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return setpoint;
    }

    public static class Command {
        public Command() {

        }

        public Command(double _leftVelocity, double _rightVelocity, double _leftAcceleration, double _rightAcceleration, double _leftFeedfowardVoltage, double _rightFeedfowardVoltage) {
            this.leftVelocity = _leftVelocity;
            this.leftAcceleration = _leftAcceleration;
            this.leftFeedfowardVoltage = _leftFeedfowardVoltage;

            this.rightVelocity = _rightVelocity;
            this.rightAcceleration = _rightAcceleration;
            this.rightFeedfowardVoltage = _rightFeedfowardVoltage;
        }

        public double leftVelocity;
        public double rightVelocity;
        
        public double leftAcceleration;
        public double rightAcceleration;

        public double leftFeedfowardVoltage;
        public double rightFeedfowardVoltage;

        public void flip() {
            double tempLeftVelocity = leftVelocity;
            leftVelocity = -rightVelocity;
            rightVelocity = -tempLeftVelocity;

            double tempLeftAcceleration = leftAcceleration;
            leftAcceleration = -rightAcceleration;
            rightAcceleration = -tempLeftAcceleration;

            double tempLeftFeedfowardVoltage = leftFeedfowardVoltage;
            leftFeedfowardVoltage = -rightFeedfowardVoltage;
            rightFeedfowardVoltage = -tempLeftFeedfowardVoltage;
        } 
    }
}