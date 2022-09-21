package frc.robot.constants;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickIOConstants{
    static Joystick joystick = new Joystick(0);

    public static double getRawAxis(int index){
        return joystick.getRawAxis(index);
    }

    public static boolean getButton(int index){
        return joystick.getRawButton(index);
    }

    public static boolean getRawButtonPressed(int index){
        return joystick.getRawButtonPressed(index);
    }

    public static boolean getRawButtonReleased(int index){
        return joystick.getRawButtonReleased(index);
    }
}