package frc.robot.autonomous.modes;

import frc.robot.autonomous.AutonBase;
import frc.robot.autonomous.actions.*;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
// import frc.lib.trajectory.TrajectoryGenerator;
// import frc.lib.trajectory.TrajectoryGenerator.Trajectories;

public class TestAuton extends AutonBase {

    // private final Trajectories trajectories = TrajectoryGenerator.getInstance().getTrajectories();

    public TestAuton() {

    }

    //Fill this with actions
    @Override
    protected void routine() {
        System.out.println("First action started at: "  + Timer.getFPGATimestamp());

        runAction(new WaitAction(1.0));//Total wait time of 1 second

        System.out.println("First action completed and second action started at: " + Timer.getFPGATimestamp());

        runAction(new ParallelAction(Arrays.asList(new WaitAction(1.0), new WaitAction(2.0))));//Total wait time of two seconds

        System.out.println("Second action completed and third action started at: " + Timer.getFPGATimestamp());

        runAction(new SeriesAction(Arrays.asList(new WaitAction(1.0), new WaitAction(2.0))));//Total wait time of three seconds
        
        System.out.println("Third action completed at: " + Timer.getFPGATimestamp());

    }
}