package frc.lib.controller;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickAxis extends Axis {
    private final Joystick joystick;
    private final int axis;

    public JoystickAxis(Joystick joystick, int axis) {
        this.joystick = joystick;
        this.axis = axis;
    }

    @Override
    public double getRaw() {
        return joystick.getRawAxis(axis);
    }
}
