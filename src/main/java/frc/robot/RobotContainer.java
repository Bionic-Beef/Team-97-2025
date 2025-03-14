// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Set;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.CoralPlacerSubsystem;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.TargetingSubsystem97;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.SetElevatorPosition;
import swervelib.SwerveInputStream;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController altXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/bionic-beef-WEEK-1"));
  public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final CoralPlacerSubsystem m_coralPlacerSubsystem = new CoralPlacerSubsystem();
  public static TargetingSubsystem97 m_TargetingSubsystem97 = new TargetingSubsystem97();

  public final RGB m_RGB = new RGB(9);

  public static final DigitalInput m_noCoralInIntakeSensor = new DigitalInput(2);
  public static boolean coralInIntake() { return !m_noCoralInIntakeSensor.get();}
  public static Trigger noCoralIntakeTrigger = new Trigger(m_noCoralInIntakeSensor::get);
  public BooleanSupplier coralBooleanSupplier = () -> !m_noCoralInIntakeSensor.get();

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1.0)
                                                            .scaleRotation(0.6)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);
  
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // drive to apriltag 6
    //NamedCommands.registerCommand("driveToBranch", drivebase.driveToPose(new Pose2d(new Translation2d(14.140, 2.425),new Rotation2d(2.0944))));
    
    //tag 22: 2
    // NamedCommands.registerCommand("driveToBranch", drivebase.driveToPose(m_TargetingSubsystem97.m_poses[9], 0.4));
    // NamedCommands.registerCommand("driveToBranchwOffset", drivebase.driveToPose(Constants.get_scoring_point_offset(m_TargetingSubsystem97.m_poses[9]), 0.4));
    // NamedCommands.registerCommand("driveToScoringPos", drivebase.driveToPose(Constants.get_scoring_point_offset(m_TargetingSubsystem97.getTargetPose()), 0.2));
    // NamedCommands.registerCommand("driveToTargetAprilTag", drivebase.driveToPose(Constants.get_scoring_point_offset(Vision.getAprilTagPose(m_TargetingSubsystem97.getTargetAprilTag(), new Transform2d())), 0.2));
    NamedCommands.registerCommand("driveToTarget", drivebase.driveToTargetPoseDeferred(0.4));
    NamedCommands.registerCommand("driveToCoral", drivebase.driveToPose(new Pose2d(new Translation2d(16.164, 0.991),new Rotation2d(2.0944))));
    NamedCommands.registerCommand("drive forward", drivebase.drivePOV(0, -1));
    NamedCommands.registerCommand("drive backward", drivebase.drivePOV(0, 1));

    //NamedCommands.registerCommand("goToL1", m_elevatorSubsystem.setGoal(ElevatorConstants.L1Height));
    //NamedCommands.registerCommand("goToL2", m_elevatorSubsystem.setGoal(ElevatorConstants.L2Height));
    NamedCommands.registerCommand("2 foot drive", new DeferredCommand(() -> drivebase.driveToPose(drivebase.getPose().transformBy(new Transform2d(0.6, 0, new Rotation2d()))), Set.of(drivebase)));
    // Elevator seems to work. CHECK AGAIN once we have elevator sensor.
    NamedCommands.registerCommand("goToL1", new SetElevatorPosition(m_elevatorSubsystem, ElevatorConstants.L1Height));
    NamedCommands.registerCommand("goToL2", new SetElevatorPosition(m_elevatorSubsystem, ElevatorConstants.L2Height));
    // coral placer and wait for coral work. time the placer runs is controlled through pathplanner with a deadline group
    // but a command that finishes after a given time would be neater.
    NamedCommands.registerCommand("runCoralPlacer", m_coralPlacerSubsystem.placerForward(1));
    NamedCommands.registerCommand("stopCoralPlacer", m_coralPlacerSubsystem.stopCoralPlacer());
    NamedCommands.registerCommand("waitForCoral", m_coralPlacerSubsystem.endWhenHasCoral(coralBooleanSupplier));
    //NamedCommands.registerCommand("autoShootCoral", m_coralPlacerSubsystem.autoShootCoral(1.0));

    // Configure the trigger bindings
    configureBindings();
    m_RGB.LEDs.setRGB(7, 128, 128, 128);
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("Leave Auto");
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // m_elevatorSubsystem.loadPreferences();
    // (Condition) ? Return-On-True : Return-on-False
    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
    //                             driveFieldOrientedAnglularVelocity :
    //                             driveFieldOrientedAnglularVelocitySim);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.back().whileTrue();
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      // drive robot relative with D pad
      driverXbox.povUp().whileTrue(drivebase.drivePOV(0, -1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povUpLeft().whileTrue(drivebase.drivePOV(1, 1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povUpRight().whileTrue(drivebase.drivePOV(-1, -1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povDown().whileTrue(drivebase.drivePOV(0, 1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povDownLeft().whileTrue(drivebase.drivePOV(1, 1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povDownRight().whileTrue(drivebase.drivePOV(-1, 1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povLeft().whileTrue(drivebase.drivePOV(1, 0, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povRight().whileTrue(drivebase.drivePOV(-1, 0, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));

      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().whileTrue(drivebase.AimAtBestTarget());
      //driverXbox.x().onTrue(m_elevatorSubsystem.goToTestSetpoint());

      // THIS WORKS
      // use a, b, y to move the elevator to L1, L2, L3
      //driverXbox.a().onTrue(m_elevatorSubsystem.setGoal(ElevatorConstants.L1Height));
      //driverXbox.b().onTrue(m_elevatorSubsystem.setGoal(ElevatorConstants.L2Height));
      driverXbox.y().onTrue(m_elevatorSubsystem.setGoal(ElevatorConstants.L3Height));

      driverXbox.a().onTrue(new SetElevatorPosition(m_elevatorSubsystem, ElevatorConstants.L1Height));
      driverXbox.b().onTrue(new SetElevatorPosition(m_elevatorSubsystem, ElevatorConstants.L2Height));


      // manually raise and lower the ellvator
      driverXbox.leftBumper().onTrue(m_elevatorSubsystem.raiseCommand(false));
      driverXbox.rightBumper().onTrue(m_elevatorSubsystem.raiseCommand(true));
      driverXbox.leftBumper().onFalse(m_elevatorSubsystem.stopC());
      driverXbox.rightBumper().onFalse(m_elevatorSubsystem.stopC());
   // use a, b, y to move the elevator to L1, L2, L3
      altXbox.a().onTrue(m_elevatorSubsystem.setGoal(ElevatorConstants.L1Height));
      altXbox.b().onTrue(m_elevatorSubsystem.setGoal(ElevatorConstants.L2Height));
      altXbox.y().onTrue(m_elevatorSubsystem.setGoal(ElevatorConstants.L3Height));

      // manually raise and lower the ellvator
      altXbox.leftBumper().onTrue(m_elevatorSubsystem.raiseCommand(false));
      altXbox.rightBumper().onTrue(m_elevatorSubsystem.raiseCommand(true));
      altXbox.leftBumper().onFalse(m_elevatorSubsystem.stopC());
      altXbox.rightBumper().onFalse(m_elevatorSubsystem.stopC());
      // NOT WORKING
      // right trigger moves the elevator one level higher, left trigger moves one level lower.
      //driverXbox.rightTrigger().onTrue(m_elevatorSubsystem.goToHigherLevel());
      //driverXbox.leftTrigger().onTrue(m_elevatorSubsystem.goToLowerLevel());

      // drives to a position by apriltag 6. untested.
      //driverXbox.start().whileTrue(drivebase.driveToPose(new Pose2d(Inches.of(530.49 + 11), Inches.of(130.17 - 19.05), new Rotation2d(2.094395102)))); //300-180 degrees

      //driverXbox.rightTrigger().whileTrue(m_elevatorSubsystem.stopC());
      //driverXbox.leftTrigger().whileTrue(m_elevatorSubsystem.setGoal(2));

      //m_elevatorSubsystem.atHeight(5, 0.1).whileTrue(Commands.print("I AM ALIVE, YAAA HAAAAA"));
      // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveCommand(() -> 1.0, () -> 0.0,() -> 0.0)
      //                         );

      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(drivebase.driveToTarget(1, new Transform2d()));//Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.leftBumper().whileTrue(drivebase.driveToPose(new Pose2d(1.0, 1.0, new Rotation2d())));//Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      
      //driverXbox.x().whileTrue(drivebase.AimAtBestTarget());
      //drive to apriltag 6
      //driverXbox.back().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(14.140, 2.425),new Rotation2d(2.0944))));
      driverXbox.back().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(Meter.of(2), Meter.of(4)), Rotation2d.fromDegrees(0.0))));
      driverXbox.x().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0.0))));


      // elevator driverXbox
      // driverXbox.rightTrigger().whileTrue(m_elevatorSubsystem.raiseElevatorCommand());
      // driverXbox.leftTrigger().whileTrue(m_elevatorSubsystem.lowerElevatorCommand());
      // driverXbox.rightTrigger().onFalse(m_elevatorSubsystem.stopElevator());
      // driverXbox.leftTrigger().onFalse(m_elevatorSubsystem.stopElevator());

      // coral placer driverXbox
      //driverXbox.y().whileTrue(m_coralPlacerSubsystem.placerForward());
      // driverXbox.x().whileTrue(m_coralPlacerSubsystem.placerReverse());
      //driverXbox.y().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());
      // driverXbox.x().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());

      // elevator altXbox
      // altXbox.rightTrigger().whileTrue(m_elevatorSubsystem.raiseElevatorCommand());
      // altXbox.leftTrigger().whileTrue(m_elevatorSubsystem.lowerElevatorCommand());
      // altXbox.rightTrigger().onFalse(m_elevatorSubsystem.stopElevator());
      // altXbox.leftTrigger().onFalse(m_elevatorSubsystem.stopElevator());

      // coral placer altXbox
      //altXbox.y().whileTrue(m_coralPlacerSubsystem.placerForward());
      //altXbox.x().whileTrue(m_coralPlacerSubsystem.placerReverse());
      //altXbox.y().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());
      //altXbox.x().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());

      // automatic coral intake
      noCoralIntakeTrigger.whileFalse(m_coralPlacerSubsystem.placerForward(0.2));
      noCoralIntakeTrigger.onTrue(m_coralPlacerSubsystem.stopCoralPlacer());

      // place coral
      driverXbox.rightTrigger().whileTrue(m_coralPlacerSubsystem.placerForward(1));
      driverXbox.leftTrigger().whileTrue(drivebase.driveToTargetPoseDeferred());
      driverXbox.rightTrigger().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());
      // driverXbox.leftTrigger().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());

      altXbox.rightTrigger().whileTrue(m_coralPlacerSubsystem.placerForward(1));
      altXbox.leftTrigger().whileTrue(m_coralPlacerSubsystem.placerReverse());
      altXbox.rightTrigger().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());
      altXbox.leftTrigger().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());
    }

  }

  public Command driveForward(){
    System.out.println("Tried to auto");
    Pose2d pose = drivebase.getPose();
    Pose2d targetPose = new Pose2d(pose.getX()+0.6, pose.getY(), pose.getRotation()); // red aliance
    return new InstantCommand(() -> drivebase.driveToPose(targetPose));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    //return drivebase.getAutonomousCommand("Test DriveToBranch");
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
