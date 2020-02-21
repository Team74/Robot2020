package frc.robot.autonomous.actions;

import java.util.ArrayList;
import java.util.List;

public class SeriesAction implements Action {

    public Action currentAction;
    public ArrayList<Action> remainingActions;

    public SeriesAction(List<Action> _actions) {
        remainingActions = new ArrayList<>(_actions);
        currentAction = null;
    }
    @Override
    public void start() {
        
    }

    @Override
    public void done() {

    }
    
    @Override
    public boolean isFinished() {
        return remainingActions.isEmpty() && currentAction == null;
    }

    @Override
    public void update() {
        if (currentAction == null) {
            if (remainingActions.isEmpty()) {
                return;
            }
            currentAction = remainingActions.remove(0);
            currentAction.start();
        }

        currentAction.update();

        if (currentAction.isFinished()) {
            currentAction.done();
            currentAction = null;
        }
        
    }
}