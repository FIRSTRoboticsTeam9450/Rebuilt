// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
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
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  public static final class RobotConstants {

    public static final String CANIVORE_BUS_NAME = "CantDrive";
    public static final String RIO_BUS_NAME = "Rio";

    public static final CANBus CANIVORE_BUS = new CANBus(CANIVORE_BUS_NAME);
    public static final CANBus RIO_BUS = new CANBus(RIO_BUS_NAME);

    public static final NeutralModeValue DEFAULT_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;

    public static final class ClimberConstants {

      public static final class DeviceIDs {
        
      }

    }

    public static final class IntakeConstants {

      public static final class DeviceIDs {
        
      }

    }

    public static final class ShooterConstants {

      public static final class DeviceIDs {
        public static final int kAngleMotorID = -1;
        public static final int kShooterMotorID = -1;
      }

      public static final class HoodConstants {
        public static final double degreesPerMotorRev = 0;
      }

    }

    public static final class TransferConstants {

      public static final class DeviceIDs {
        public static final int kDrivingMotorID = -1;
      }

    }

    public static final class TurretConstants {

      public static final class DeviceIDs {
        public static final int kMotorID = -1;
        public static final int kAbsEncoderID = -1;
      }

      public static final double ROTOR_TO_SENSOR_RATIO = 0.001890359168;
      public static final double SENSOR_TO_MECHANISM_RATIO = 0.4565217391;

      public static final double TURRET_OFFSET_TO_ZERO = 135;

      public static final double TOTAL_ROTATION_DEGREES = 270;
      public static final double TOTAL_ROTATION_RADIANS = 3 / (2 * Math.PI);
      public static final double DEGREE_CHANGE_PER_MOTOR_REVOLUTION = 1.5;
      public static final double ABS_ENCDR_POS_TO_ANGLE = 480;

    }
    public static final class ZoneConstants {
      public static final class ZoneConstantsWelded {
        public static final double MAJOR_ZONE_X_LOW = 0;
        public static final double MAJOR_ZONE_X_HIGH = 182.11;
        public static final double MAJOR_ZONE_Y_LOW = 0;
        public static final double MAJOR_ZONE_Y_HIGH = 317.69;

        public static final double MID_ZONE_X_LOW = 182.11;
        public static final double MID_ZONE_X_HIGH = 651.22;
        public static final double MID_ZONE_1_Y_LOW = 158.84;
        public static final double MID_ZONE_1_Y_HIGH = 317.69;

        public static final double MID_ZONE_2_Y_LOW = 0;
        public static final double MID_ZONE_2_Y_HIGH = 158.84;

        public static final double MID_ZONE_2_DEADZONE_Y_LOW = 0;
        public static final double MID_ZONE_2_DEADZONE_Y_HIGH = 0;

        public static final double TRENCH_12_DEADZONE_X_LOW = 0;
        public static final double TRENCH_12_DEADZONE_X_HIGH = 0;
        public static final double TRENCH_1_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_1_DEADZONE_Y_HIGH = 0;

        public static final double TRENCH_2_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_2_DEADZONE_Y_HIGH = 0;

        public static final double TRENCH_34_DEADZONE_X_LOW = 0;
        public static final double TRENCH_34_DEADZONE_X_HIGH = 0;
        public static final double TRENCH_3_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_3_DEADZONE_Y_HIGH = 0;

        public static final double TRENCH_4_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_4_DEADZONE_Y_HIGH = 0;
      }
      public static final class ZoneConstantsAndyMark {
        public static final double MAJOR_ZONE_X_LOW = 0;
        public static final double MAJOR_ZONE_X_HIGH = 181.56;
        public static final double MAJOR_ZONE_Y_LOW = 0;
        public static final double MAJOR_ZONE_Y_HIGH = 316.65;

        public static final double MID_ZONE_X_LOW = 181.56;
        public static final double MID_ZONE_X_HIGH = 650.12;
        public static final double MID_ZONE_1_Y_LOW = 158.32;
        public static final double MID_ZONE_1_Y_HIGH = 316.64;

        public static final double MID_ZONE_2_Y_LOW = 0;
        public static final double MID_ZONE_2_Y_HIGH = 158.32;

        public static final double MID_ZONE_2_DEADZONE_Y_LOW = 0;
        public static final double MID_ZONE_2_DEADZONE_Y_HIGH = 0;

        public static final double TRENCH_12_DEADZONE_X_LOW = 0;
        public static final double TRENCH_12_DEADZONE_X_HIGH = 0;
        public static final double TRENCH_1_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_1_DEADZONE_Y_HIGH = 0;

        public static final double TRENCH_2_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_2_DEADZONE_Y_HIGH = 0;

        public static final double TRENCH_34_DEADZONE_X_LOW = 0;
        public static final double TRENCH_34_DEADZONE_X_HIGH = 0;
        public static final double TRENCH_3_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_3_DEADZONE_Y_HIGH = 0;

        public static final double TRENCH_4_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_4_DEADZONE_Y_HIGH = 0;
      }
      public static final class ZoneTargets {
        public static final Pose2d SHOOTING_TARGET = new Pose2d();
        public static final Pose2d FERRY_TOP_TARGET = new Pose2d();
        public static final Pose2d FERRY_BOTTOM_TARGET = new Pose2d();
      }
    }

    
    public static final class LimelightConstants {

      public static final class FrontLeft {
        public static final String NAME = "limelight-fl";
        public static final double LOCATION_X = 0;
        public static final double LOCATION_Y = 0;
        public static final double LOCATION_Z = 0;
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = 0;
      }

      public static final class FrontRight {
        public static final String NAME = "limelight-fr";
        public static final double LOCATION_X = 0;
        public static final double LOCATION_Y = 0;
        public static final double LOCATION_Z = 0;
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = 0;
      }

      public static final class BackLeft {
        public static final String NAME = "limelight-bl";
        public static final double LOCATION_X = 0;
        public static final double LOCATION_Y = 0;
        public static final double LOCATION_Z = 0;
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = 0;
      }

      public static final class BackRight {
        public static final String NAME = "limelight-br";
        public static final double LOCATION_X = 0;
        public static final double LOCATION_Y = 0;
        public static final double LOCATION_Z = 0;
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = 0;
      }

      public static final class Pillar {
        public static final String NAME = "limelight-pillar";
        public static final double LOCATION_X = 0;
        public static final double LOCATION_Y = 0;
        public static final double LOCATION_Z = 0;
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = 0;
      }

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
