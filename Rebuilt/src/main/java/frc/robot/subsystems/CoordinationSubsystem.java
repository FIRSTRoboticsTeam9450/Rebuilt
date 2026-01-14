package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoordinationSubsystem extends SubsystemBase {
    

    public static class GameData {
        private String gameData;
        private boolean isBlueHubActive = true;

        public void updateData() {
            gameData = DriverStation.getGameSpecificMessage();
            if (gameData.length() > 0) {
                if (gameData.charAt(0) == 'B') {
                    isBlueHubActive = true;
                } else {
                    isBlueHubActive = false;
                }
            }
        }

        public boolean isBlueHubActive() {
            return isBlueHubActive;
        }
    }
}
