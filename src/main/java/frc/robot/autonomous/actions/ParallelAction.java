package frc.robot.autonomous.actions;

import java.util.ArrayList;
import java.util.List;

public class ParallelAction implements Action {

    public final ArrayList<Action> actions;

    public ParallelAction(List<Action> _actions) {
        actions = new ArrayList<>(_actions);
    }

    public void start() {
       for (Action action : actions) {
           action.start();
       } 
    }

    public void done() {
        for (Action action : actions) {
            action.start();
        }
    }

    public boolean isFinished() {
        for (Action action : actions) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }

    public void update() {
        for (Action action : actions) {
            action.update();
        }
    }
}