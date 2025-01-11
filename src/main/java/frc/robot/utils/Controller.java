/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class Controller extends XboxController {

    public JoystickButton leftBumper;
    public JoystickButton rightBumper;

    public JoystickButton leftTrigger;
    public JoystickButton rightTrigger;

    public JoystickButton yButton;
    public JoystickButton aButton;
    public JoystickButton bButton;
    public JoystickButton xButton;

    public JoystickButton startButton;
    public JoystickButton selectButton;

    public JoystickButton leftStickButton;
    public JoystickButton rightStickButton;

    public POVButton dpadUp;
    public POVButton dpadDown;
    public POVButton dpadLeft;
    public POVButton dpadRight;

    public Controller(int port) {
        super(port);

        leftTrigger = new JoystickButton(this, XboxController.Axis.kLeftTrigger.value);
        rightTrigger = new JoystickButton(this, XboxController.Axis.kRightTrigger.value);

        leftBumper = new JoystickButton(this, XboxController.Button.kLeftBumper.value);
        rightBumper = new JoystickButton(this, XboxController.Button.kRightBumper.value);

        yButton = new JoystickButton(this, XboxController.Button.kY.value);
        aButton = new JoystickButton(this, XboxController.Button.kA.value);
        bButton = new JoystickButton(this, XboxController.Button.kB.value);
        xButton = new JoystickButton(this, XboxController.Button.kX.value);

        startButton = new JoystickButton(this, XboxController.Button.kStart.value);
        selectButton = new JoystickButton(this, XboxController.Button.kBack.value);

        leftStickButton = new JoystickButton(this, XboxController.Button.kLeftStick.value);
        rightStickButton = new JoystickButton(this, XboxController.Button.kRightStick.value);

        dpadUp = new POVButton(this, 0);
        dpadDown = new POVButton(this, 180);
        dpadLeft = new POVButton(this, 270);
        dpadRight = new POVButton(this, 90);
    }
}
