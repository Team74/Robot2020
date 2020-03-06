package frc.robot;

public interface Updateable {
    public void handleInput();
    public void update(double dt);
    public void dashboard();
}