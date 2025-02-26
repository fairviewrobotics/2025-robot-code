/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.constants.ScoringConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ButtonBoardSubsystemTest {
    private GenericHID mockHid;
    private ButtonBoardSubsystem buttonBoardSubsystem;

    @BeforeEach
    void setUp() {
        mockHid = mock(GenericHID.class);

        buttonBoardSubsystem = new ButtonBoardSubsystem(mockHid);
    }

    @Test
    void testButtonPressSetsCorrectHeight() {
        int buttonIndex = 14;
        BooleanEvent mockEvent = mock(BooleanEvent.class);
        when(mockHid.getRawButton(buttonIndex)).thenReturn(true);

        buttonBoardSubsystem.periodic();

        ScoringConstants.ScoringHeights expectedHeight =
                ScoringConstants.ScoringHeights.valueOf("L2");
        assertEquals(expectedHeight, buttonBoardSubsystem.getCurrentHeight());
    }

    @Test
    void testButtonPressSetsCorrectPose() {
        int buttonIndex = 3;
        BooleanEvent mockEvent = mock(BooleanEvent.class);
        when(mockHid.getRawButton(buttonIndex)).thenReturn(true);

        buttonBoardSubsystem.periodic();

        Pose2d expectedPose = ScoringConstants.CORAL_POSITIONS[buttonIndex - 1];
        assertEquals(expectedPose, buttonBoardSubsystem.getCurrentPose());
    }

    @Test
    void testButtonMappingForBlueAlliance() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        int buttonIndex = 3;
        when(mockHid.getRawButton(buttonIndex)).thenReturn(true);

        buttonBoardSubsystem = new ButtonBoardSubsystem(mockHid);
        buttonBoardSubsystem.periodic();

        Pose2d expectedPose = ScoringConstants.CORAL_POSITIONS[buttonIndex - 1 + 12];
        assertEquals(expectedPose, buttonBoardSubsystem.getCurrentPose());

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1); // Set back to red
        DriverStationSim.notifyNewData();
    }
}
