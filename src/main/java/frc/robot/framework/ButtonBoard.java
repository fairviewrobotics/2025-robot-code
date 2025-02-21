package frc.robot.framework;
import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBoard {

    private final GenericHID buttonBoard;
    private String coralPositionString;
    private String coralHeightString;

    private CoralQueue.CoralPosition currentCoralPosition;
    private CoralQueue.CoralPosition lastCoralPosition;

    public ButtonBoard(GenericHID buttonBoard) {
        this.buttonBoard = buttonBoard;
        coralPositionString = "";
        coralHeightString = "";
    }

    public void periodic() {
        for(int i = 0; i<=3; i++) {
            if (buttonBoard.getRawButtonPressed(i)) {
                coralHeightString = "H" + i;
                break;
            }
        }

        for (int i = 4; i <= 16; i++) {
            if (buttonBoard.getRawButtonPressed(i)) {
                coralPositionString = String.valueOf(i);
                break;
            }
        }

        if (lastCoralPosition != null && lastCoralPosition.toString().equals(currentCoralPosition.toString())) return;

        coralHeightString = "";
        coralPositionString = "";
        lastCoralPosition = currentCoralPosition;
        currentCoralPosition = CoralQueue.getCoralPosition(currentCoralPosition + coralHeightString);
    }
}
