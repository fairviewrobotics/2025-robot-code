/* Black Knights Robotics (C) 2025 */
package org.blackknights;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        System.setProperty("log4j.configurationFile", "log4j2.xml");
        System.setProperty("org.blackknights.isTest", "false");

        RobotBase.startRobot(Robot::new);
    }
}
