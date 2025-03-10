// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

//import java.security.DigestInputStream;
import java.util.HashMap;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.units.DistanceUnit;
//import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
import swervelib.math.Matter;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (100) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(17.6);
  public static final HashMap<Integer, String> IDS_TO_NAME = new HashMap<Integer, String>();
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants{
    public static final int elevatorLeaderID = (Integer) 13;
    public static final int elevatorFollowerID = (Integer) 14;
    public static final double kElevatorKp = 11.5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;//1.6047; 

    public static final double kElevatorkS = 0.0;//1964; // volts (V)
    public static final double kElevatorkV = 0; //3.894; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0;//0.173; // volt per acceleration (V/(m/s²))
    public static final double kElevatorkG = 0;//0.91274; // volts (V)

    public static final double kElevatorGearing    = (115/6);
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.5);
    public static final double kCarriageMass       = 6; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final Distance kStartingHeightSim = Meters.of(0);
    public static final Distance kMinElevatorHeight = Meters.of(0.0);
    public static final Distance kMaxElevatorHeight = Meters.of(0.852475);

    // these are the height above L1 in meters
    public static final double L1Height = 0; // L1 is 1ft, 6in off the ground
    public static final double L2Height = 0.352425; // L2 is 2ft, 7 and 7/8in off the ground
    public static final double L3Height = 0.695;//0.752475; // L3 is 3ft, 11 and 5/8in off the ground
    public static final double L4Height = 1.3716; // L4 is 6ft, 0in off the ground

    public static double kElevatorRampRate = 1.5;
    public static int    kElevatorCurrentLimit = 30;
    public static double kMaxVelocity = Meters.of(0.1).per(Second).in(MetersPerSecond); //2
    public static double kMaxAcceleration = Meters.of(0.1).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static double kTolerance = 0.01; //0.1

    // dashboard preferences
    public static final String L1SetpointKey = "L1Position";
    public static final double defaultL1Setpoint = 0.3;
  }

  public static class CoralPlacerConstants{
    public static final int placerMotorID = (Integer) 15;
    public static final int sensorDIO = 3;
  }

  // From 2342 - https://github.com/FRCTeamPhoenix/Build2025

  public static final class VisionConstants {
    public static final String LOW_BACK_CAMERA_NAME = "back_arducam";
    public static final String LEFT_CAMERA_NAME = "left_arducam";
    public static final String RIGHT_CAMERA_NAME = "right_arducam";

    public static final Transform3d FRONT_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.2944),
                Units.inchesToMeters(8.9822),
                Units.inchesToMeters(12.125)),
            new Rotation3d(0, 0.0, Units.degreesToRadians(-35)));

    public static final Transform3d FRONT_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.2944),
                Units.inchesToMeters(-8.9822),
                Units.inchesToMeters(12.125)),
            new Rotation3d(0, 0.0, Units.degreesToRadians(35)));

    public static final Transform3d LOW_BACK_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-12.73),
                Units.inchesToMeters(11.286),
                Units.inchesToMeters(7.89)),
            new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(180)));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.1;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] CAMERA_STD_DEV_FACTORS =
        new double[] {
          1.0, // Camera 0
          1.0, // Camera 1
          1.0 // Camera 2
        };

    // Multipliers to apply for MegaTag 2 observations
    public static final double LINEAR_STD_DEV_MEGATAG2_FACTOR =
        0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_MEGATAG2_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class FieldConstants {
    public static final Pose3d[] ZONE_ALIGN_BLUE_POSES =
        new Pose3d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(21).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(22).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(17).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(18).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(19).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(20).orElse(new Pose3d())
        };
    public static final Pose3d[] ZONE_ALIGN_RED_POSES =
        new Pose3d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(7).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(6).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(11).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(10).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(9).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(8).orElse(new Pose3d())
        };
    public static final double REEF_BUFFER = DriveConstants.DRIVE_BASE_RADIUS + 0.4;
    public static final Transform2d REEF_BUFFER_TRANSFORM =
        new Transform2d(REEF_BUFFER, 0, Rotation2d.k180deg);

    public static final Pose2d BLUE_REEF_CENTER = new Pose2d(4.489323, 4.0259, new Rotation2d());
    public static final Pose2d RED_REEF_CENTER = new Pose2d(13.058902, 4.0259, new Rotation2d());

    public static final double X_LIMIT = 2.75;
    public static final double Y_LIMIT = 3.5;
    public static final double SLOPE = 0.61261261261261;
    public static final Transform2d[] ZONE_TRANSFORMS =
        new Transform2d[] {
          new Transform2d(-X_LIMIT, Y_LIMIT, Rotation2d.kZero),
          new Transform2d(X_LIMIT, Y_LIMIT, Rotation2d.kZero),
          new Transform2d(X_LIMIT, -Y_LIMIT, Rotation2d.kZero),
          new Transform2d(-X_LIMIT, -Y_LIMIT, Rotation2d.kZero),
          new Transform2d(-X_LIMIT, Y_LIMIT, Rotation2d.kZero),
          new Transform2d(-X_LIMIT, SLOPE * X_LIMIT, Rotation2d.kZero),
          new Transform2d(-1.14, 0.68, Rotation2d.kZero),
          new Transform2d(0, 1.32, Rotation2d.kZero),
          new Transform2d(0, Y_LIMIT, Rotation2d.kZero),
          new Transform2d(0, 1.32, Rotation2d.kZero),
          new Transform2d(1.14, 0.68, Rotation2d.kZero),
          new Transform2d(X_LIMIT, SLOPE * X_LIMIT, Rotation2d.kZero),
          new Transform2d(1.14, 0.68, Rotation2d.kZero),
          new Transform2d(1.14, -0.68, Rotation2d.kZero),
          new Transform2d(X_LIMIT, -SLOPE * X_LIMIT, Rotation2d.kZero),
          new Transform2d(1.14, -0.68, Rotation2d.kZero),
          new Transform2d(0, -1.32, Rotation2d.kZero),
          new Transform2d(0, -Y_LIMIT, Rotation2d.kZero),
          new Transform2d(0, -1.32, Rotation2d.kZero),
          new Transform2d(-1.14, -0.68, Rotation2d.kZero),
          new Transform2d(-X_LIMIT, -SLOPE * X_LIMIT, Rotation2d.kZero),
          new Transform2d(-1.14, -0.68, Rotation2d.kZero),
          new Transform2d(-1.14, 0.68, Rotation2d.kZero),
        };

    // 6.468
    public static final double BRANCH_BUFFER = DriveConstants.DRIVE_BASE_RADIUS + 0.12;
    public static final Transform2d LEFT_BRANCH =
        new Transform2d(BRANCH_BUFFER, Units.inchesToMeters(-6.468), Rotation2d.k180deg);
    public static final Transform2d RIGHT_BRANCH =
        new Transform2d(BRANCH_BUFFER, Units.inchesToMeters(6.468), Rotation2d.k180deg);

    public static final Pose2d[] BLUE_PLAYER_STATIONS =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(13).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(12).orElse(new Pose3d()).toPose2d(),
        };
    public static final Pose2d[] RED_PLAYER_STATIONS =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(1).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(2).orElse(new Pose3d()).toPose2d(),
        };

    public static final double STATION_BUFFER = 0.606;
    public static final Transform2d CENTER_PLAYER_STATION =
        new Transform2d(STATION_BUFFER, 0, Rotation2d.k180deg);

    public static final Transform2d PROCESSOR_BUFFER =
        new Transform2d(DriveConstants.DRIVE_BASE_RADIUS + 0.12, 0, Rotation2d.k180deg);
    public static final Pose2d BLUE_PROCESSOR =
        VisionConstants.TAG_LAYOUT
            .getTagPose(16)
            .orElse(new Pose3d())
            .toPose2d()
            .plus(PROCESSOR_BUFFER);
    public static final Pose2d RED_PROCESSOR =
        VisionConstants.TAG_LAYOUT
            .getTagPose(3)
            .orElse(new Pose3d())
            .toPose2d()
            .plus(PROCESSOR_BUFFER);

    public static final Pose2d[] ALGAE_BLUE_POSES =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(21).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(22).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(17).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(18).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(19).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(20).orElse(new Pose3d()).toPose2d(),
        };
    public static final int[] ALGAE_BLUE_STATES = new int[] {6, 7, 6, 7, 6, 7};
    public static final Pose2d[] ALGAE_RED_POSES =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(7).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(6).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(11).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(10).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(9).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(8).orElse(new Pose3d()).toPose2d(),
        };
    public static final int[] ALGAE_RED_STATES = new int[] {7, 6, 7, 6, 7, 6};
  }

  // HACK - TODO - Fix these
  public static final class DriveConstants {
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.5);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.941);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(29.0 - (2.625 * 2));
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(28.0 - (2.625 * 2));
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(90);
    public static final double ROBOT_MOI = 5.278;
    public static final double WHEEL_COF = 1.2;
    public static final double SLIP_CURRENT = 40;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    // public static final double[] DEV_ENCODER_OFFSETS = {2.888, -2.246 + Math.PI, -2.976, -2.745};
    public static final double[] COMP_ENCODER_OFFSETS = {
      1.969 + Math.PI, 2.481 + Math.PI, 0.82 + Math.PI, -0.7 + Math.PI
    };

    public static final double[] ENCODER_OFFSETS = COMP_ENCODER_OFFSETS;
  }


  public static final class AutoConstants {
    public static final PathConstraints CONSTRAINTS =
        new PathConstraints(
            DriveConstants.MAX_LINEAR_SPEED,
            3.0,
            Units.degreesToRadians(720),
            Units.degreesToRadians(1080));

    public static final Constraints LINEAR_CONSTRAINTS =
        new Constraints(DriveConstants.MAX_LINEAR_SPEED, 4);
    public static final Constraints ANGLE_CONSTRAINTS =
        new Constraints(Units.degreesToRadians(720), Units.degreesToRadians(1080));

    public static final Constraints FINE_LINEAR_CONSTRAINTS = new Constraints(2, 0.75);
    public static final Constraints FINE_ANGLE_CONSTRAINTS =
        new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(540));

    public static final Transform2d PATHING_BUFFER = new Transform2d(1, 0, Rotation2d.k180deg);

    public static final Transform2d LEFT_STATION_PATHING_BUFFER =
        new Transform2d(1.5, 1, Rotation2d.k180deg);

    public static final Transform2d RIGHT_STATION_PATHING_BUFFER =
        new Transform2d(1.5, -1, Rotation2d.k180deg);

    public static final Pose3d[] BLUE_REEF_POSES =
        new Pose3d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(21).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(22).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(17).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(18).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(19).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(20).orElse(new Pose3d())
        };
    public static final Pose3d[] RED_REEF_POSES =
        new Pose3d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(10).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(9).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(8).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(7).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(6).orElse(new Pose3d()),
          VisionConstants.TAG_LAYOUT.getTagPose(11).orElse(new Pose3d())
        };

    public static final Pose2d[] PATHING_BLUE_PLAYER_STATIONS =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(13).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(12).orElse(new Pose3d()).toPose2d()
        };

    public static final Pose2d[] PATHING_RED_PLAYER_STATIONS =
        new Pose2d[] {
          VisionConstants.TAG_LAYOUT.getTagPose(1).orElse(new Pose3d()).toPose2d(),
          VisionConstants.TAG_LAYOUT.getTagPose(2).orElse(new Pose3d()).toPose2d()
        };
  }
}
