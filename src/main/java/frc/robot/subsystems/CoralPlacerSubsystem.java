package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants.CoralPlacerConstants;

public class CoralPlacerSubsystem extends SubsystemBase {
    private SparkMax placerMotor = new SparkMax(CoralPlacerConstants.placerMotorID, SparkLowLevel.MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();
    @SuppressWarnings("unused")
    private final double intakeSpeed = 0.2;
    private final double placerSpeed = 0.5;

    public CoralPlacerSubsystem() {
        config.idleMode(IdleMode.kBrake);
        placerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command placerForward(double speed) {
        return Commands.run(() -> {
            placerMotor.set(speed);
        });
    }

    public Command placerReverse() {
        return Commands.run(() -> {
            placerMotor.set(-placerSpeed);
        });
    }

    public Command stopCoralPlacer() {
        return Commands.run(() -> {
            placerMotor.set(0);
        });
    }
}
