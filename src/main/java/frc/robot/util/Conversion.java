package frc.robot.util;

public class Conversion {
    public static double ticksToMeters(double ticks, double wheelDiamater) {
        return (ticks / 2048.0) * ((wheelDiamater * Math.PI) / 39.37);
    }

    // CHECK TO MAKE SURE IT SHOULD BE < 180 and NOT <= 180
    public static double normalizeGyro(double currAngle) {
        if (currAngle < -180) {
            return 180 + (currAngle + 180);
        } else if (currAngle > 180) {
            return -180 + (currAngle - 180);
        } else {
            return currAngle;
        }
    }
}
