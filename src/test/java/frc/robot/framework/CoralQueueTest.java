/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CoralQueueTest {
    private CoralQueue coralQueue;

    @BeforeEach
    void setUp() {
        coralQueue = new CoralQueue();
    }

    @Test
    void testAddToQueue() {
        coralQueue.addToQueue("R0H1"); // Example position
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(12.0)), 1.0);
        assertEquals(expected, coralQueue.getNext());

        coralQueue.addToQueue("B2H4"); // Example position
        expected =
                new CoralQueue.CoralPosition(
                        new Pose2d(3.0, 12.0, Rotation2d.fromDegrees(9.0)), 5.0);
        assertEquals(expected, coralQueue.getNext());

        coralQueue.addToQueue("B10H4");
        expected =
                new CoralQueue.CoralPosition(
                        new Pose2d(13.0, 24.0, Rotation2d.fromDegrees(45.0)), 5.0);
        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testListToQueue() {
        coralQueue.listToQueue("B10H4,R0H1,B2H4");
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        new Pose2d(13.0, 24.0, Rotation2d.fromDegrees(45.0)), 5.0);
        assertEquals(expected, coralQueue.getNext());
        expected =
                new CoralQueue.CoralPosition(
                        new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(12.0)), 1.0);
        assertEquals(expected, coralQueue.getNext());
        expected =
                new CoralQueue.CoralPosition(
                        new Pose2d(3.0, 12.0, Rotation2d.fromDegrees(9.0)), 5.0);
        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testSkipNextValue() {
        coralQueue.listToQueue("B10H4,R0H1");
        coralQueue.skipNextValue();
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(12.0)), 1.0);
        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testAddToFront() {
        coralQueue.listToQueue("B10H4,R0H1");
        coralQueue.getNext();
        coralQueue.addToFront();
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                    new Pose2d(13.0, 24.0, Rotation2d.fromDegrees(45.0)), 5.0);
        assertEquals(expected, coralQueue.getNext());
    }
}
