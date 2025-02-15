package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem {
    private SparkMax elevatorLeader = new SparkMax(ElevatorConstants.elevatorLeaderID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax elevatorFollower = new SparkMax(ElevatorConstants.elevatorFollowerID, SparkLowLevel.MotorType.kBrushless);
    private SparkMaxConfig followerConfig = new SparkMaxConfig();

    private double elevatorSpeed = 0.2;

    public ElevatorSubsystem(){
        followerConfig.follow(ElevatorConstants.elevatorLeaderID, true);
        elevatorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command raiseElevatorCommand(){
        return Commands.run(() -> {
            elevatorLeader.set(elevatorSpeed);
        });
    }

    public Command lowerElevatorCommand(){
        return Commands.run(() -> {
            elevatorLeader.set(-elevatorSpeed);
        });
    }

    public Command stopElevator(){
        return Commands.run(() -> {
            elevatorLeader.set(0);
        });
    }
}
