package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static double getTriggers(XboxController controller) {
        return Math.pow(controller.getLeftTriggerAxis()-controller.getRightTriggerAxis(), 3);
    }

    public static double getLeftAxis(XboxController controller) {
        return Math.pow(controller.getLeftX(), 3);
    }

    public static double getRightAxis(XboxController controller) {
        return Math.pow(controller.getRightY(), 3);
    }
}
