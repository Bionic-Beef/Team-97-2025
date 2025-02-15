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
import frc.robot.Constants.CoralPlacerConstants;

public class CoralPlacerSubsystem extends SubsystemBase{
    private SparkMax placerMotor = new SparkMax(CoralPlacerConstants.placerMotorID, SparkLowLevel.MotorType.kBrushless);
    private final double placerSpeed = 0.2;

    public CoralPlacerSubsystem(){}

    public Command placerForward(){
        return Commands.run(() -> {
            placerMotor.set(placerSpeed);
        });
    }

    public Command placerReverse(){
        return Commands.run(() -> {
            placerMotor.set(-placerSpeed);
        });
    }

    public Command stopCoralPlacer(){
        return Commands.run(() -> {
            placerMotor.set(0);
        });
    }
}
