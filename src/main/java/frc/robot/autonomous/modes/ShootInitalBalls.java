package frc.robot.autonomous.modes;

import frc.robot.autonomous.AutonBase;
import frc.robot.autonomous.actions.*;

import edu.wpi.first.wpilibj.Timer;
// import frc.lib.trajectory.TrajectoryGenerator;
// import frc.lib.trajectory.TrajectoryGenerator.Trajectories;

public class ShootInitalBalls extends AutonBase {

    // private final Trajectories trajectories = TrajectoryGenerator.getInstance().getTrajectories();

    public ShootInitalBalls() {

    }

    //Fill this with actions
    @Override
    protected void routine() {
        System.out.println("First action started at: "  + Timer.getFPGATimestamp());

        runAction(new ScoreBallsAction(10.0, 3));//Score First 3 balls

        System.out.println("First Action Finished Second Action Scarted");

        runAction(new DriveDistanceAction(Double.POSITIVE_INFINITY, -90.0));//Drive 75 Encoder Ticks Straight fowards

        runAction(new StopAction(Double.POSITIVE_INFINITY));

    }
}