package ca.team4308.absolutelib.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.BooleanSupplier;

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

        public static int LSButton = 9;
        public static int RSButton = 10;
    }

    public final CommandXboxController controller;

    public final Trigger A;
    public final Trigger B;
    public final Trigger X;
    public final Trigger Y;

    public final Trigger LB;
    public final Trigger RB;
    public final Trigger LeftTrigger;
    public final Trigger RightTrigger;

    public final Trigger Start;
    public final Trigger Back;

    public final Trigger LSButton;
    public final Trigger RSButton;

    public final Trigger Up;
    public final Trigger UpRight;
    public final Trigger Right;
    public final Trigger DownRight;
    public final Trigger Down;
    public final Trigger DownLeft;
    public final Trigger Left;
    public final Trigger UpLeft;

    public XBoxWrapper(int port) {
        this.controller = new CommandXboxController(port);

        this.A = controller.a(); 
        this.B = controller.b();
        this.X = controller.x();
        this.Y = controller.y();
        
        this.LB = controller.leftBumper();
        this.RB = controller.rightBumper();

        this.LeftTrigger = controller.leftTrigger(0.2);
        this.RightTrigger = controller.rightTrigger(0.2);

        this.LSButton = controller.leftStick();
        this.RSButton = controller.rightStick();

        this.Start = controller.start();
        this.Back = controller.back();

        this.Up = new POVButton(controller.getHID(), 0);
        this.UpRight = new POVButton(controller.getHID(), 45);
        this.Right = new POVButton(controller.getHID(), 90);
        this.DownRight = new POVButton(controller.getHID(), 135);
        this.Down = new POVButton(controller.getHID(), 180);
        this.DownLeft = new POVButton(controller.getHID(), 225);
        this.Left = new POVButton(controller.getHID(), 270);
        this.UpLeft = new POVButton(controller.getHID(), 315);
    }

    public double getLeftX() {
        return controller.getLeftX(); // 0
    }

    public double getLeftY() {
        return controller.getLeftY(); // 1
    }

    public double getRightX() {
        return controller.getRightX(); // 4
    }

    public double getRightY() {
        return controller.getRightY(); // 5
    }

    public double getLeftTrigger() {
        return controller.getLeftTriggerAxis(); // 2
    }

    public double getRightTrigger() {
        return controller.getRightTriggerAxis(); // 3
    }

    public BooleanSupplier getAButtonPressed() {
        return () -> controller.getHID().getAButtonPressed();
    }

    public BooleanSupplier getBButtonPressed() {
        return () -> controller.getHID().getBButtonPressed();
    }

    public BooleanSupplier getXButtonPressed() {
        return () -> controller.getHID().getXButtonPressed();
    }

    public BooleanSupplier getYButtonPressed() {
        return () -> controller.getHID().getYButtonPressed();
    }

    public void setLeftRumble(Double value) {
        controller.getHID().setRumble(RumbleType.kLeftRumble, value);
    }

    public void setRightRumble(Double value) {
        controller.getHID().setRumble(RumbleType.kLeftRumble, value);
    }
}
