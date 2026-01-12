// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int DRIVER_CONTROLLER_PORT = 0;

  public static final class RobotConstants {

    public static final String CANIVORE_BUS_NAME = "CantDrive";
    public static final String RIO_BUS_NAME = "Rio";

    public static final CANBus CANIVORE_BUS = new CANBus(CANIVORE_BUS_NAME);
    public static final CANBus RIO_BUS = new CANBus(RIO_BUS_NAME);

    public static final NeutralModeValue DEFAULT_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;

    public static final class ClimberConstants {

      public static final class MotorIDs {
        
      }

    }

    public static final class IntakeConstants {

      public static final class MotorIDs {
        
      }

    }

    public static final class ShooterConstants {

      public static final class MotorIDs {
        
      }

    }

    public static final class TransferConstants {

      public static final class MotorIDs {
        public static final int drivingMotor = -1;
      }

    }

    public static final class TurretConstants {

      public static final class MotorIDs {
        public static final int angleMotor = -1;
      }

      public static final double TOTAL_TURRET_ROTATION_DEGREES = 270;
      public static final double TOTAL_TURRET_ROTATION_RADIANS = 3 / (2 * Math.PI);

    }

  }

  public static final class FieldConstants {

    public static boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    public static AprilTagFieldLayout FIELD_LAYOUT;

    static {
      // FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026Rebuilt);
      FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public static final class ImportantFieldPoses {
      public static final Translation2d RED_HUB_CENTER = new Translation2d(4.6256, 4.0345);
      public static final Translation2d BLUE_HUB_CENTER = new Translation2d(11.9154, 4.0345);
    }

    public static Pose3d getTag3dPose(int tagID) {
      // if (tagID < AprilTagIDs.RED_LEFT_CORAL_STATION || tagID > AprilTagIDs.BLUE_REEF_LEFT_BARGE) {
      //   throw new IllegalArgumentException("Error: Given Tag ID must be between 1 and 22 (inclusive).");
      // }

      return FIELD_LAYOUT.getTagPose(tagID).orElseThrow(() -> {
        final String errorMsg = String.format("getTagPose called for unexpected tag, %d", tagID);
        return new RuntimeException(errorMsg);
      });
    }
  }
}
