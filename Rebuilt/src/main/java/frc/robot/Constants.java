// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static final String DRIVEBASE_CANBUS_NAME = "CantDrive";
    public static final String MAIN_SYSTEMS_CANBUS_NAME = "CantShoot";
    public static final String RIO_BUS_NAME = "Rio";

    public static final CANBus DRIVEBASE_CANBUS = new CANBus(DRIVEBASE_CANBUS_NAME);
    public static final CANBus MAIN_SYSTEMS_CANBUS = new CANBus(MAIN_SYSTEMS_CANBUS_NAME);
    public static final CANBus RIO_BUS = new CANBus(RIO_BUS_NAME);

    public static final boolean ENABLE_FOC = true;

    public static final NeutralModeValue DEFAULT_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;

    public static final class LEDConstants {
      public static final class DeviceIDs {
        public static final int kCANDleID = 45;
      }
    }

    public static final class ClimberConstants {

      public static final class DeviceIDs {
        public static final int kElevator = 40;
        public static final int kTelescope = 41;
      }

      public static final class ClimbPositions {
        public static final double kElevatorFullyUp = -0.5;
        public static final double kTelescopeFullyUp = 40;
        public static final double kElevatorDown = -128.35;
        public static final double kTelescopeClipped = 0;
      }

    }

    public static final class IntakeConstants {

      public static final class DeviceIDs {
        public static final int kAngleMotorID = 20;
        public static final int kWheelMotorID = 21;
      }

    }

    public static final class ShooterConstants {

      public static final class DeviceIDs {
        public static final int kAngleMotorID = 35;
        public static final int kLeftShooterMotorID = 36;
        public static final int kRightShooterMotorID = 37;
      }

      public static final class HoodConstants {
        public static final double degreesPerMotorRev = 2.0571428571429;
      }

    }

    public static final class TransferConstants {

      public static final class DeviceIDs {
        public static final int kDrivingMotorID = 25;
      }

    }

    public static final class TurretConstants {

      public static final class DeviceIDs {
        public static final int kMotorID = 30;
        public static final int kAbsEncoderID = 31;
      }

      //7in off from center
      public static final Transform2d OFFSET_WRT_CENTER_OF_ROBOT = new Transform2d(new Translation2d(-0.1778, 0), new Rotation2d(0));

      public static final double ENCODER_MAGNET_OFFSET = 0.35009765625;

      //Lantern motot to turret gear reduction 161:1 (1/161)
      public static final double ROTOR_TO_SENSOR_RATIO = 0.0625850; //Both Cycoidals: 0.001827990359168
      public static final double SENSOR_TO_MECHANISM_RATIO = 0.933333;// Non-Lantern: 0.4565217391 * 1.06; Cycoildal-Lanter: 0.311279 * 0.9849444

      public static final double TURRET_OFFSET_TO_ZERO = 179.5; 
      public static final double TURRET_OFFSET_TO_TARGET = -5; //V1: 5

      public static final double TOTAL_ROTATION_DEGREES = 300;
      public static final double TOTAL_ROTATION_RADIANS = Math.toRadians(TOTAL_ROTATION_DEGREES);
      /** Unused, just a spot that saves the data incase we need to quickly check it */
      public static final double DEGREE_CHANGE_PER_MOTOR_REVOLUTION = 2.24; //Lantern: 2.24
      public static final double ABS_ENCDR_POS_TO_ANGLE = 360;
    }
    
    public static final class LimelightConstants {

      public static final boolean isAtNorthField = false;

      public static final class PipelineIndexs{
        //NorthField
        public static final int NF_CLOSE = 0;
        public static final int NF_MED = 1;
        public static final int NF_FAR = 2;

        //Lab
        public static final int LAB_CLOSE = 3;
        public static final int LAB_MED = 4;
        public static final int LAB_FAR = 5;

        //Comp
        public static final int COMP_CLOSE = 6;
        public static final int COMP_MED = 7;
        public static final int COMP_FAR = 8;
      }

      public static final class FrontLeft {
        public static final String NAME = "limelight-fl";
        public static final double LOCATION_X = -0.2862961;
        public static final double LOCATION_Y = 0.28271978;
        public static final double LOCATION_Z = 0.21090128;
        public static final double PITCH = 28.1;
        public static final double ROLL = 0;
        public static final double YAW = 45;
        public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
      }

      public static final class FrontRight {
        public static final String NAME = "limelight-fr";
        public static final double LOCATION_X = 0.2862961;
        public static final double LOCATION_Y = 0.28271978;
        public static final double LOCATION_Z = 0.21090128;
        public static final double PITCH = 28.1;
        public static final double ROLL = 0;
        public static final double YAW = -45;
        public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
      }

      public static final class BackLeft {
        public static final String NAME = "limelight-bl";
        public static final double LOCATION_X = -0.2862961;
        public static final double LOCATION_Y = -0.28271978;
        public static final double LOCATION_Z = 0.21090128;
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = 135;
        public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
      }

      public static final class BackRight {
        public static final String NAME = "limelight-br";
        public static final double LOCATION_X = 0.2862961;
        public static final double LOCATION_Y = -0.28271978;
        public static final double LOCATION_Z = 0.21090128;
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = -135;
        public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
      }

    }

  }

  public static final class FieldConstants {

    public static boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    public static AprilTagFieldLayout FIELD_LAYOUT;

    static {
      FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
      FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    /**
     * Ferry Poses are from blue alliance driver box perspective
     */
    public static final class ImportantFieldPoses {
      public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6256, 4.0345);
      public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9154, 4.0345);

      public static final Translation2d FERRY_TOP_TARGET = new Translation2d(0.455, 6.95); // Used to be .455, 6.95
      public static final Translation2d FERRY_BOTTOM_TARGET = new Translation2d(0.960,1.971);

      /* ========= HARD CODED POSES ======== */
      public static final Translation2d BLUE_LEFT_FERRY_TARGET = new Translation2d();
      public static final Translation2d BLUE_RIGHT_FERRY_TARGET = new Translation2d();

      public static final Translation2d RED_LEFT_FERRY_TARGET = new Translation2d();
      public static final Translation2d RED_RIGHT_FERRY_TARGET = new Translation2d();
    }

    public static final class AprilTagIDs {
      public static final int RED_BOTTOM_TRENCH_NEUTRAL_SIDE = 1;
      public static final int RED_BOTTOM_HUB_NEUTRAL_SIDE = 2;
      public static final int RED_NEUTRAL_HUB_SIDE_LOWER = 3;
      public static final int RED_NEUTRAL_HUB_SIDE_HIGHER = 4;
      public static final int RED_TOP_HUB_NEUTRAL_SIDE = 5;
      public static final int RED_TOP_TRENCH_NEUTRAL_SIDE = 6;
      public static final int RED_TOP_TRENCH_ALLIANCE_SIDE = 7;
      public static final int RED_TOP_HUB_ALLIANCE_SIDE = 8;
      public static final int RED_ALLIANCE_HUB_SIDE_HIGHER = 9;
      public static final int RED_ALLIANCE_HUB_SIDE_LOWER = 10;
      public static final int RED_BOTTOM_HUB_ALLIANCE_SIDE = 11;
      public static final int RED_BOTTOM_TRENCH_ALLIANCE_SIDE = 12;
      public static final int RED_OUTPOST_LOWER = 13;
      public static final int RED_OUTPOST_HIGHER = 14;
      public static final int RED_TOWER_LOWER = 15;
      public static final int RED_TOWER_HIGHER = 16;

      public static final int BLUE_TOP_TRENCH_NEUTRAL_SIDE = 17;
      public static final int BLUE_TOP_HUB_NEUTRAL_SIDE = 18;
      public static final int BLUE_NEUTRAL_HUB_SIDE_HIGHER = 19;
      public static final int BLUE_NEUTRAL_HUB_SIDE_LOWER = 20;
      public static final int BLUE_BOTTOM_HUB_NEUTRAL_SIDE = 21;
      public static final int BLUE_BOTTOM_TRENCH_NEUTRAL_SIDE = 22;
      public static final int BLUE_BOTTOM_TRENCH_ALLIANCE_SIDE = 23;
      public static final int BLUE_BOTTOM_HUB_ALLIANCE_SIDE = 24;
      public static final int BLUE_ALLIANCE_HUB_SIDE_LOWER = 25;
      public static final int BLUE_ALLIANCE_HUB_SIDE_HIGHER = 26;
      public static final int BLUE_TOP_HUB_ALLIANCE_SIDE = 27;
      public static final int BLUE_TOP_TRENCH_ALLIANCE_SIDE = 28;
      public static final int BLUE_OUTPOST_HIGHER = 29;
      public static final int BLUE_OUTPOST_LOWER = 30;
      public static final int BLUE_TOWER_HIGHER = 31;
      public static final int BLUE_TOWER_LOWER = 32;
    }

    public static Pose3d getTag3dPose(int tagID) {
      return FIELD_LAYOUT.getTagPose(tagID).orElseThrow(() -> {
        final String errorMsg = String.format("getTagPose called for unexpected tag, %d", tagID);
        return new RuntimeException(errorMsg);
      });
    }

    public static final Map<Pose2d, Integer> aprilTagPoseToTagIDsMap1_10 = Map.of(
      getTag3dPose(1).toPose2d(), 1,
      getTag3dPose(2).toPose2d(), 2,
      getTag3dPose(3).toPose2d(), 3,
      getTag3dPose(4).toPose2d(), 4,
      getTag3dPose(5).toPose2d(), 5,
      getTag3dPose(6).toPose2d(), 6,
      getTag3dPose(7).toPose2d(), 7,
      getTag3dPose(8).toPose2d(), 8,
      getTag3dPose(9).toPose2d(), 9,
      getTag3dPose(10).toPose2d(), 10
    );

    public static final Map<Pose2d, Integer> aprilTagPoseToTagIDsMap11_20 = Map.of(
      getTag3dPose(11).toPose2d(), 11,
      getTag3dPose(12).toPose2d(), 12,
      getTag3dPose(13).toPose2d(), 13,
      getTag3dPose(14).toPose2d(), 14,
      getTag3dPose(15).toPose2d(), 15,
      getTag3dPose(16).toPose2d(), 16,
      getTag3dPose(17).toPose2d(), 17,
      getTag3dPose(18).toPose2d(), 18,
      getTag3dPose(19).toPose2d(), 19,
      getTag3dPose(20).toPose2d(), 20
    );

    public static final Map<Pose2d, Integer> aprilTagPoseToTagIDsMap21_30 = Map.of(
      getTag3dPose(21).toPose2d(), 21,
      getTag3dPose(22).toPose2d(), 22,
      getTag3dPose(23).toPose2d(), 23,
      getTag3dPose(24).toPose2d(), 24,
      getTag3dPose(25).toPose2d(), 25,
      getTag3dPose(26).toPose2d(), 26,
      getTag3dPose(27).toPose2d(), 27,
      getTag3dPose(28).toPose2d(), 28,
      getTag3dPose(29).toPose2d(), 29,
      getTag3dPose(30).toPose2d(), 30
    );

    public static final Map<Pose2d, Integer> aprilTagPoseToTagIDsMap31_32 = Map.of(
      getTag3dPose(31).toPose2d(), 31,
      getTag3dPose(32).toPose2d(), 32
    );

    /**BLUE                      RED
     * -----------------------------
     * |    1 | 2          5 | 6   |
     * |     HUB            HUB    |
     * |    3 | 4          7 | 8   |
     * -----------------------------
     */
    public static final class TrenchManeuvering {

      public static enum TrenchLocationWRTBlueOrigin {
        CLOSE_LEFT,
        CLOSE_RIGHT,
        FAR_LEFT,
        FAR_RIGHT
      }
      
      public static final Translation2d trenchOne = new Translation2d(2.618, 7.435);
      public static final Translation2d trenchTwo = new Translation2d(6.618, 7.435);
      public static final Translation2d trenchThree = new Translation2d(2.618, 0.627);
      public static final Translation2d trenchFour = new Translation2d(6.618, 0.627);

      public static final Translation2d trenchFive = new Translation2d(9.618, 7.435);
      public static final Translation2d trenchSix = new Translation2d(14.618, 7.435);
      public static final Translation2d trenchSeven = new Translation2d(9.618, 0.627);
      public static final Translation2d trenchEight = new Translation2d(14.618, 0.627);

    }

    public static final class ZoneConstants {

      public static final class ZoneConstantsWelded {
        public static final Translation2d RED_CORNER = new Translation2d(16.54, 8.069326);
        public static final double MAJOR_ZONE_X_LOW = 0;
        public static final double MAJOR_ZONE_X_HIGH = 4.625594; // inches 182.11;
        public static final double MAJOR_ZONE_Y_LOW = 0;
        public static final double MAJOR_ZONE_Y_HIGH = 8.06936; // inches 317.69;

        public static final double MID_ZONE_X_LOW = MAJOR_ZONE_X_HIGH;//182.11;
        public static final double MID_ZONE_X_HIGH = 16.540988; // inches 651.22;
        public static final double MID_ZONE_1_Y_LOW = -0.5; //meters: 4.034536 inches 158.84; 4.3 meters if wanting preference to outpost side
        public static final double MID_ZONE_1_Y_HIGH = 8.069326; // inches 317.69;

        public static final double MID_ZONE_2_Y_LOW = 0;
        public static final double MID_ZONE_2_Y_HIGH = MID_ZONE_1_Y_LOW;//158.84;

        public static final double MID_ZONE_2_DEADZONE_X_LOW = 11.915394;
        public static final double MID_ZONE_2_DEADZONE_X_HIGH = 16.540988;
        public static final double MID_ZONE_2_DEADZONE_Y_LOW = 3.437636;
        public static final double MID_ZONE_2_DEADZONE_Y_HIGH = 4.631436;

        public static final double HARD_TRENCH_12_DEADZONE_X_LOW = 4.549394 - .762; // 4.535424
        public static final double HARD_TRENCH_12_DEADZONE_X_HIGH = 4.701794 + .762; // 4.687824
        public static final double SOFT_TRENCH_12_DEADZONE_X_LOW = 4.549394 - 1.524; // 4.535424
        public static final double SOFT_TRENCH_12_DEADZONE_X_HIGH = 4.701794 + 1.524; // 4.687824
        public static final double TRENCH_1_DEADZONE_Y_LOW = 6.37203;
        public static final double TRENCH_1_DEADZONE_Y_HIGH = 8.069326;

        public static final double TRENCH_2_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_2_DEADZONE_Y_HIGH = 1.6970375;

        public static final double HARD_TRENCH_34_DEADZONE_X_LOW = 11.839194 - .762;
        public static final double HARD_TRENCH_34_DEADZONE_X_HIGH = 11.99 + .762;
        public static final double SOFT_TRENCH_34_DEADZONE_X_LOW = 11.839194 - 1.524;
        public static final double SOFT_TRENCH_34_DEADZONE_X_HIGH = 11.99 + 1.524;
        public static final double TRENCH_3_DEADZONE_Y_LOW = TRENCH_1_DEADZONE_Y_LOW;
        public static final double TRENCH_3_DEADZONE_Y_HIGH = TRENCH_1_DEADZONE_Y_HIGH;

        public static final double TRENCH_4_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_4_DEADZONE_Y_HIGH = TRENCH_2_DEADZONE_Y_HIGH;

        // TODO: FIX THE TRENCH DEADZONE VALUES TO ACTUALLY BE WELDED AND NOT ANDYMARK
        // public static final double TRENCH_12_DEADZONE_X_LOW = 4.549394; // Currently just covering the trench, no extra deadzone area
        // public static final double TRENCH_12_DEADZONE_X_HIGH = 4.701794;
        // public static final double TRENCH_1_DEADZONE_Y_LOW = 6.37203;
        // public static final double TRENCH_1_DEADZONE_Y_HIGH = 8.069326;

        // public static final double TRENCH_2_DEADZONE_Y_LOW = 0;
        // public static final double TRENCH_2_DEADZONE_Y_HIGH = 1.6970375;

        // public static final double TRENCH_34_DEADZONE_X_LOW = 11.839194;
        // public static final double TRENCH_34_DEADZONE_X_HIGH = 11.99;
        // public static final double TRENCH_3_DEADZONE_Y_LOW = TRENCH_1_DEADZONE_Y_LOW;
        // public static final double TRENCH_3_DEADZONE_Y_HIGH = TRENCH_1_DEADZONE_Y_HIGH;

        // public static final double TRENCH_4_DEADZONE_Y_LOW = 0;
        // public static final double TRENCH_4_DEADZONE_Y_HIGH = TRENCH_2_DEADZONE_Y_HIGH;
      }
      public static final class ZoneConstantsAndyMark {
        public static final Translation2d RED_CORNER = new Translation2d(16.513048, 8.042656);

        public static final double MAJOR_ZONE_X_LOW = 0;
        public static final double MAJOR_ZONE_X_HIGH = 4.611624; // inches 181.56;
        public static final double MAJOR_ZONE_Y_LOW = 0;
        public static final double MAJOR_ZONE_Y_HIGH = 8.04291; // inches 316.65;

        public static final double MID_ZONE_X_LOW = MAJOR_ZONE_X_HIGH; //181.56;
        public static final double MID_ZONE_X_HIGH = 16.513048; //650.12;
        public static final double MID_ZONE_1_Y_LOW = 4.021328; //158.32;
        public static final double MID_ZONE_1_Y_HIGH = 8.042656; //316.64;

        public static final double MID_ZONE_2_Y_LOW = 0;
        public static final double MID_ZONE_2_Y_HIGH = MID_ZONE_1_Y_LOW; //158.32;

        public static final double MID_ZONE_2_DEADZONE_X_LOW = 11.901;
        public static final double MID_ZONE_2_DEADZONE_X_HIGH = 16.513048;
        public static final double MID_ZONE_2_DEADZONE_Y_LOW = 3.424428;
        public static final double MID_ZONE_2_DEADZONE_Y_HIGH = 4.618228;

        public static final double HARD_TRENCH_12_DEADZONE_X_LOW = 4.535424 - .762; // 4.535424
        public static final double HARD_TRENCH_12_DEADZONE_X_HIGH = 4.687824 + .762; // 4.687824
        public static final double SOFT_TRENCH_12_DEADZONE_X_LOW = 4.535424 - 1.524; // 4.535424
        public static final double SOFT_TRENCH_12_DEADZONE_X_HIGH = 4.687824 + 1.524; // 4.687824
        public static final double TRENCH_1_DEADZONE_Y_LOW = 6.3456185;
        public static final double TRENCH_1_DEADZONE_Y_HIGH = 8.042656;

        public static final double TRENCH_2_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_2_DEADZONE_Y_HIGH = 1.6970375;

        public static final double HARD_TRENCH_34_DEADZONE_X_LOW = 11.825224 - .762;
        public static final double HARD_TRENCH_34_DEADZONE_X_HIGH = 11.977624 + .762;
        public static final double SOFT_TRENCH_34_DEADZONE_X_LOW = 11.825224 - 1.524;
        public static final double SOFT_TRENCH_34_DEADZONE_X_HIGH = 11.977624 + 1.524;
        public static final double TRENCH_3_DEADZONE_Y_LOW = TRENCH_1_DEADZONE_Y_LOW;
        public static final double TRENCH_3_DEADZONE_Y_HIGH = TRENCH_1_DEADZONE_Y_HIGH;

        public static final double TRENCH_4_DEADZONE_Y_LOW = 0;
        public static final double TRENCH_4_DEADZONE_Y_HIGH = TRENCH_2_DEADZONE_Y_HIGH;
      }
      public static final class ZoneTargets {
        public static final Translation2d SHOOTING_TARGET = ImportantFieldPoses.BLUE_HUB_CENTER;
        public static final Translation2d FERRY_TOP_TARGET = ImportantFieldPoses.FERRY_TOP_TARGET;
        public static final Translation2d FERRY_BOTTOM_TARGET = ImportantFieldPoses.FERRY_BOTTOM_TARGET;
      }
    }

  }
}
