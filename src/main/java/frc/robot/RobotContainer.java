// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.CoralPlacerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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
                                                                                "swerve/bionic-beef-NERD-event"));
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final CoralPlacerSubsystem m_coralPlacerSubsystem = new CoralPlacerSubsystem();

  public static final DigitalInput m_noCoralInIntakeSensor = new DigitalInput(0);
  public static Trigger noCoralIntakeTrigger = new Trigger(m_noCoralInIntakeSensor::get);

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
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.5)
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
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
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
    // // (Condition) ? Return-On-True : Return-on-False
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
      driverXbox.povUpLeft().whileTrue(drivebase.drivePOV(1, -1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povUpRight().whileTrue(drivebase.drivePOV(-1, -1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povDown().whileTrue(drivebase.drivePOV(0, 1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povDownLeft().whileTrue(drivebase.drivePOV(1, 1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povDownRight().whileTrue(drivebase.drivePOV(-1, 1, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povLeft().whileTrue(drivebase.drivePOV(1, 0, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));
      driverXbox.povRight().whileTrue(drivebase.drivePOV(-1, 0, () -> ((driverXbox.button(5).getAsBoolean() ? 0.1 : 0) + (driverXbox.button(6).getAsBoolean() ? -0.1 : 0))));


      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveCommand(() -> 1.0, () -> 0.0,() -> 0.0)
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(drivebase.driveToTarget(1, new Transform2d()));//Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.leftBumper().whileTrue(drivebase.driveToPose(new Pose2d(1.0, 1.0, new Rotation2d())));//Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().whileTrue(drivebase.AimAtBestTarget());

      // elevator driverXbox
      driverXbox.rightTrigger().whileTrue(m_elevatorSubsystem.raiseElevatorCommand());
      driverXbox.leftTrigger().whileTrue(m_elevatorSubsystem.lowerElevatorCommand());
      driverXbox.rightTrigger().onFalse(m_elevatorSubsystem.stopElevator());
      driverXbox.leftTrigger().onFalse(m_elevatorSubsystem.stopElevator());

      // coral placer driverXbox
      driverXbox.y().whileTrue(m_coralPlacerSubsystem.placerForward());
      driverXbox.x().whileTrue(m_coralPlacerSubsystem.placerReverse());
      driverXbox.y().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());
      driverXbox.x().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());

      // elevator altXbox
      altXbox.rightTrigger().whileTrue(m_elevatorSubsystem.raiseElevatorCommand());
      altXbox.leftTrigger().whileTrue(m_elevatorSubsystem.lowerElevatorCommand());
      altXbox.rightTrigger().onFalse(m_elevatorSubsystem.stopElevator());
      altXbox.leftTrigger().onFalse(m_elevatorSubsystem.stopElevator());

      // coral placer altXbox
      altXbox.y().whileTrue(m_coralPlacerSubsystem.placerForward());
      altXbox.x().whileTrue(m_coralPlacerSubsystem.placerReverse());
      altXbox.y().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());
      altXbox.x().onFalse(m_coralPlacerSubsystem.stopCoralPlacer());

      // automatic coral intake
      noCoralIntakeTrigger.onFalse(m_coralPlacerSubsystem.placerForward());
      noCoralIntakeTrigger.onTrue(m_coralPlacerSubsystem.stopCoralPlacer());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
