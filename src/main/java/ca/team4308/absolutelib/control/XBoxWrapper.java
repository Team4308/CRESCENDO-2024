package ca.team4308.absolutelib.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class XBoxWrapper {
    public static class XBoxMapping {
        public static int A = 1;
        public static int B = 2;
        public static int X = 3;
        public static int Y = 4;

        public static int LB = 5;
        public static int RB = 6;

        public static int Start = 7;
        public static int Back = 8;

        public static int LeftStickButton = 9;
        public static int RightStickButton = 10;
    }

    public final Joystick joystick;

    public final JoystickButton A;
    public final JoystickButton B;
    public final JoystickButton X;
    public final JoystickButton Y;

    public final JoystickButton LB;
    public final JoystickButton RB;

    public final JoystickButton Start;
    public final JoystickButton Back;

    public final JoystickButton LeftStickButton;
    public final JoystickButton RightStickButton;

    public final POVButton pov0;
    public final POVButton pov45;
    public final POVButton pov90;
    public final POVButton pov135;
    public final POVButton pov180;
    public final POVButton pov225;
    public final POVButton pov270;
    public final POVButton pov315;

    public XBoxWrapper(int port) {
        this.joystick = new Joystick(port);

        this.A = new JoystickButton(joystick, XBoxMapping.A);
        this.B = new JoystickButton(joystick, XBoxMapping.B);
        this.X = new JoystickButton(joystick, XBoxMapping.X);
        this.Y = new JoystickButton(joystick, XBoxMapping.Y);
        
        this.LB = new JoystickButton(joystick, XBoxMapping.LB);
        this.RB = new JoystickButton(joystick, XBoxMapping.RB);
        this.LeftStickButton = new JoystickButton(joystick, XBoxMapping.LeftStickButton);
        this.RightStickButton = new JoystickButton(joystick, XBoxMapping.RightStickButton);

        this.Start = new JoystickButton(joystick, XBoxMapping.Start);
        this.Back = new JoystickButton(joystick, XBoxMapping.Back);

        this.pov0 = new POVButton(joystick, 0);
        this.pov45 = new POVButton(joystick, 45);
        this.pov90 = new POVButton(joystick, 90);
        this.pov135 = new POVButton(joystick, 135);
        this.pov180 = new POVButton(joystick, 180);
        this.pov225 = new POVButton(joystick, 225);
        this.pov270 = new POVButton(joystick, 270);
        this.pov315 = new POVButton(joystick, 315);
    }

    public double getLeftX() {
        return joystick.getRawAxis(0);
    }

    public double getLeftY() {
        return joystick.getRawAxis(1);
    }

    public double getRightX() {
        return joystick.getRawAxis(4);
    }

    public double getRightY() {
        return joystick.getRawAxis(5);
    }

    public double getLeftTrigger() {
        return joystick.getRawAxis(2);
    }

    public double getRightTrigger() {
        return joystick.getRawAxis(3);
    }

    public void setLeftRumble(Double value) {
        joystick.setRumble(RumbleType.kLeftRumble, value);
    }

    public void setRightRumble(Double value) {
        joystick.setRumble(RumbleType.kRightRumble, value);
    }
}
