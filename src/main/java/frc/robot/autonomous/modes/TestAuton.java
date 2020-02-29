package frc.robot.autonomous.modes;

import frc.robot.autonomous.AutonBase;
import frc.robot.autonomous.actions.*;

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

        runAction(new TurnAngleAction(Double.POSITIVE_INFINITY, 90.0));//Turn 90s to the right

        System.out.println("First action completed and second action started at: " + Timer.getFPGATimestamp());

        // runAction(new WaitAction(3.0));//Wait 3.0 seconds

        System.out.println("Second action completed and third action started at: " + Timer.getFPGATimestamp());

        runAction(new TurnAngleAction(Double.POSITIVE_INFINITY, 0.0));//Turn 90s to the left, back to the starting heading
        
        System.out.println("Third action completed at: " + Timer.getFPGATimestamp());

    }
}