package frc.robot.autonomous.actions;

/*
Template for all actions
*/

public interface Action {
    double startTime = 0.0;
    double maxRunTime = 0.0;

    //Containts code to start action
    void start();

    //Run when action is done
    void done();

    //Define some criteria for when action is done here 
    boolean isFinished();
    
    //All code that neds to be updated goes here
    void update();
}