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

    //TODO: adjust these so positions are correct
    public static final Pose2d kOrigin = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPathEnd = new Pose2d(new Translation2d(50.0, 0.0), Rotation2d.fromDegrees(0.0));

    //TODO: adjust these so positions are correct
    //NOTE: These should be able to be rotated 180 degrees to get the ones on the other side of the field.
    //Locations of the 3 in line balls in tench defined as first ball nearest to the driver station
    public static final Pose2d kFirstTrenchBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kSecondTrenchBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kThirdTrenchBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));
    //Location to pick up side by side ball on opposite side of the trench
    public static final Pose2d k45TrenchBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));

    //TODO: adjust these so positions are correct
    //NOTE: These should be able to be rotated 180 degrees to get ones on other side of the field.
    //Locations of the 5 balls in the rendezvous defined as first ball nearest to the trench
    public static final Pose2d kFirstRendezvousBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kSecondRendezvousBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kThirdRendezvousBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kFourthRendezvousBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kFifthRendezvousBall = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0.0));

    //TODO: Define shooting positions


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
        public final MirroredTrajectory mirroredTestPath;

        private Trajectories() {
            testPath = getTestPath();
            mirroredTestPath = new MirroredTrajectory(getTestPath());

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kOrigin);
            waypoints.add(kTestPathEnd);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAcceleration, kMaxVoltage);
        }
    }



}