package frc.lib.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//Since the Talon remebers the previous setpoint, this class removes uneccessary set command by overriding the TalonSRX set fundtion
public class TalonOverride extends TalonSRX {

    private double lastValue;
    private ControlMode lastControlMode;

    public TalonOverride(int canID) {
        super(canID);    
    }

    public void stopMotor() {
        set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void set(ControlMode _controlMode, double _value) {
        if (lastValue != _value || lastControlMode != _controlMode) {
            lastValue = _value;
            lastControlMode = _controlMode;
            super.set(_controlMode, _value);
        } 
    }
}