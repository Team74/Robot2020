package frc.robot.autonomous.modes;

import frc.robot.autonomous.AutonBase;
import frc.robot.autonomous.actions.*;

import edu.wpi.first.wpilibj.Timer;
// import frc.lib.trajectory.TrajectoryGenerator;
// import frc.lib.trajectory.TrajectoryGenerator.Trajectories;

public class GetOffLineMode extends AutonBase {

    // private final Trajectories trajectories = TrajectoryGenerator.getInstance().getTrajectories();

    public GetOffLineMode() {

    }

    //Fill this with actions
    @Override
    protected void routine() {
        System.out.println("First action started at: "  + Timer.getFPGATimestamp());

        runAction(new DriveDistanceAction(Double.POSITIVE_INFINITY, 100));//Drive 500 Encoder Ticks Straight Foward

        runAction(new StopAction(Double.POSITIVE_INFINITY));

    }
}