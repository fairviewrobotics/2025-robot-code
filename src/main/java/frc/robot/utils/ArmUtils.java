/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

public class ArmUtils {
    /**
     * converts encoder value to radians
     *
     * @param encoder
     * @return encoder value in radians
     */
    public static double encoderToRad(double encoder) {
        return encoder * ((Math.PI / 2) - ((8 * Math.PI) / 180)) / 23.170 + (Math.PI * 8) / 180;
    }
}
