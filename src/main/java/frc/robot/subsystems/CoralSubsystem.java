package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


public class CoralSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  public static final class ElevatorSetpoints {
    public static final int kFeederStation = 0;
    public static final int kLevel1 = 0;
    public static final int kLevel2 = 0;
    public static final int kLevel3 = 100;
    public static final int kLevel4 = 150;
  }


  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax elevatorMotor =
      new SparkMax(ElevatorConstants.elevatorLeaderID, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private SparkMax followerSparkMax = new SparkMax(ElevatorConstants.elevatorFollowerID, MotorType.kBrushless);

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;

  
 
  // Mechanism2d setup for subsystem
  private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;

  public CoralSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
        SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .follow(ElevatorConstants.elevatorLeaderID)
        .inverted(true);

    followerSparkMax.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    elevatorMotor.configure(
      config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    // Display mechanism2d

    // Zero arm and elevator encoders on initialization
    elevatorEncoder.setPosition(0);
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              // armCurrentTarget = ArmSetpoints.kFeederStation;
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              break;
            case kLevel1:
              // armCurrentTarget = ArmSetpoints.kLevel1;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kLevel2:
              // armCurrentTarget = ArmSetpoints.kLevel2;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:
              // armCurrentTarget = ArmSetpoints.kLevel3;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
              // armCurrentTarget = ArmSetpoints.kLevel4;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }
        });
  }



  @Override
  public void periodic() {
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());


  }

}