// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.security.DigestInputStream;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final HashMap IDS_TO_NAME = new HashMap<Integer, String>();
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
    public static final double DEADBAND        = 0.2;
    public static final double LEFT_Y_DEADBAND = 0.2;
    public static final double RIGHT_X_DEADBAND = 0.2;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants{
    public static final int elevatorLeaderID = (Integer) 13;
    public static final int elevatorFollowerID = (Integer) 14;
    public static final double kElevatorKp = 4.5;
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
    public static final Distance kMaxElevatorHeight = Meters.of(2);

    // these are the height above L1 in meters
    public static final double L1Height = 0; // L1 is 1ft, 6in off the ground
    public static final double L2Height = 0.352425; // L2 is 2ft, 7 and 7/8in off the ground
    public static final double L3Height = 0.752475; // L3 is 3ft, 11 and 5/8in off the ground
    public static final double L4Height = 1.3716; // L4 is 6ft, 0in off the ground


    public static double kElevatorRampRate = 0.5;
    public static int    kElevatorCurrentLimit = 30;
    public static double kMaxVelocity = Meters.of(1).per(Second).in(MetersPerSecond); //2
    public static double kMaxAcceleration = Meters.of(4).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static double kTolerance = 0.05; //0.1

  }

  public static class CoralPlacerConstants{
    public static final int placerMotorID = (Integer) 15;
    public static final int sensorDIO = 3;
  }
}
