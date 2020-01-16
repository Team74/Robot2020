package frc.lib.trajectory;

import frc.robot.DrivePlanner;

import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraints;

import frc.lib.utils.geometry.Pose2d;
import frc.lib.utils.geometry.Pose2dWithCurvature;
import frc.lib.utils.geometry.Translation2d;
import frc.lib.utils.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


import edu.wpi.first.wpilibj.Timer;

/*
This is the class we will use to generate inital trajectories we want to follow.
It will contain all the functions and variables neccessary to generate those paths, ie. waypoints
Paths will be stored in a subclass
*/

public class TrajectoryGenerator {
    private static TrajectoryGenerator kInstance = null;

    private static final double kMaxVelocity = 0.0;//inches per second
    private static final double kMaxAcceleration = 0.0;//inches per second^2
    private static final double kMaxVoltage = 0.0;//Volts
    
    private final DrivePlanner mDrivePlanner = DrivePlanner.getInstance();
    private Trajectories mTrajectories = null;
    
    public static TrajectoryGenerator getInstance() {
        if (kInstance == null) {
            kInstance = new TrajectoryGenerator();
        }
        return kInstance;
    }

    public TrajectoryGenerator() {
        generateTrajectories();
    }

    public void generateTrajectories() {
        if (mTrajectories == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories... ");
            mTrajectories = new Trajectories();
            System.out.println("Finished generating trajectories in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public Trajectories getTrajectories() {
        return mTrajectories;
    } 

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean _revearsed, 
            final List<Pose2d> _waypoints, 
            final List<TimingConstraints<Pose2dWithCurvature>> _constraints,
            double _maxVelocity,
            double _maxAcceleration,
            double _maxVoltage) {
        return mDrivePlanner.generateTrajectory(
            _revearsed,
            _waypoints, 
            _constraints, 
            _maxVelocity, 
            _maxAcceleration,
            _maxVoltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean _revearsed,
            final List<Pose2d> _waypoints,
            final List<TimingConstraints<Pose2dWithCurvature>> _constraints,
            double _startVelocity,
            double _endVelocity,
            double _maxVelocity,
            double _maxAcceleration,
            double _maxVoltage){
        return mDrivePlanner.generateTrajectory(_revearsed,
        _waypoints, 
        _constraints, 
        _startVelocity, 
        _endVelocity, 
        _maxVelocity, 
        _maxAcceleration, 
        _maxVoltage);
    }
    
    /*
    List of important positions
    Origin is the center of the alliance station wall
    If you are standing in on the origin facing the center of the field, you would be at point (0,0) with heading 0
    To your left would be positive y, straight ahead would be positive x
    x and y are defined in inches
    */

    //TODO: adjust these so positions are correct and rework how some of the crossline points are calculated.
    public static final Pose2d kOrigin = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPathEnd = new Pose2d(new Translation2d(50.0, 0.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kLeftStart = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kCenterStart = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kLeftHighStart = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kCrossLine = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kLeftCrossLine = kLeftStart.transformBy(kCrossLine);
    public static final Pose2d kCenterCrossLine = kCenterStart.transformBy(kCrossLine);

    public class Trajectories {

        public class MirroredTrajectory {

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;

            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> _left) {
                this.left = _left;
                this.right = TrajectoryUtil.mirrorTimed(_left, _left.defaultVelocity());
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean _isLeft) {
                return _isLeft ? this.left : this.right;
            }
        }

        public final Trajectory<TimedState<Pose2dWithCurvature>> testPath;

        public final MirroredTrajectory dismount;

        public final MirroredTrajectory sideCrossLine;
        public final MirroredTrajectory centerCrossLine;

        private Trajectories() {
            testPath = getTestPath();
            dismount = new MirroredTrajectory(getDismount());
            sideCrossLine = new MirroredTrajectory(getSideCrossLine());
            centerCrossLine = new MirroredTrajectory(getCenterCrossLine());

        }

        //TODO: kMaxDeceleration can be changed for differant paths
        //      kDefaultVelocity can be changed for differant paths
        //      kSlowdownChunks can be changed for differant paths
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOrigin);
            waypoints.add(kTestPathEnd);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAcceleration, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideCrossLine() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftStart);
            waypoints.add(kLeftCrossLine);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAcceleration, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCenterCrossLine() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStart);
            waypoints.add(kCenterCrossLine);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAcceleration, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDismount() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftHighStart);
            waypoints.add(kLeftStart);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAcceleration, kMaxVoltage);
        }
    }



}