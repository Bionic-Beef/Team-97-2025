package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TargetingSubsystem97 extends SubsystemBase {
    private double m_ID = 42;

    public Pose2d[] m_bluePoses = {
      new Pose2d(1, 2,new Rotation2d(0)),
      new Pose2d(5.23, 3.04, Rotation2d.fromDegrees(120)),
      new Pose2d(4.92, 2.86, Rotation2d.fromDegrees(120)),
      new Pose2d(4.00, 2.89, Rotation2d.fromDegrees(60)),
      new Pose2d(3.72, 3.05, Rotation2d.fromDegrees(60)),
      new Pose2d(3.27, 3.85, Rotation2d.fromDegrees(0)),
      new Pose2d(3.27, 4.20, Rotation2d.fromDegrees(0)),
      new Pose2d(3.74, 5.02, Rotation2d.fromDegrees(-60)),
      new Pose2d(4.02, 5.19, Rotation2d.fromDegrees(-60)),
      new Pose2d(4.94, 5.18, Rotation2d.fromDegrees(-120)),
      new Pose2d(5.20, 5.03, Rotation2d.fromDegrees(-120)),
      new Pose2d(5.71, 4.25, Rotation2d.fromDegrees(180)),
      new Pose2d(5.71, 3.84, Rotation2d.fromDegrees(180))
    };

    public Pose2d[] m_redPoses = {
      new Pose2d(1, 2,new Rotation2d(0)),
      new Pose2d(5.23, 3.04, Rotation2d.fromDegrees(120)),
      new Pose2d(4.92, 2.86, Rotation2d.fromDegrees(120)),
      new Pose2d(4.00, 2.89, Rotation2d.fromDegrees(60)),
      new Pose2d(3.72, 3.05, Rotation2d.fromDegrees(60)),
      new Pose2d(3.27, 3.85, Rotation2d.fromDegrees(0)),
      new Pose2d(3.27, 4.20, Rotation2d.fromDegrees(0)),
      new Pose2d(3.74, 5.02, Rotation2d.fromDegrees(-60)),
      new Pose2d(4.02, 5.19, Rotation2d.fromDegrees(-60)),
      new Pose2d(4.94, 5.18, Rotation2d.fromDegrees(-120)),
      new Pose2d(5.20, 5.03, Rotation2d.fromDegrees(-120)),
      new Pose2d(5.71, 4.25, Rotation2d.fromDegrees(180)),
      new Pose2d(5.71, 3.84, Rotation2d.fromDegrees(180))
    };

    public Pose2d[] m_poses;

    public Pose2d targetPose = m_poses[12];

    public double getTargetID() { return m_ID; }

    public void setTargetID(double ID) { 
      m_ID = ID % 12;
      while(m_ID <= 0)
        m_ID += 12;
      SmartDashboard.putNumber("liveTarget2", m_ID);
      targetPose = m_poses[(int)m_ID];
    }
    public Pose2d getTargetPose() { return targetPose; }

    public double[] getTargetPoseAsDoubles() { double[] a = {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()}; return a;};

    public TargetingSubsystem97() {
        SmartDashboard.putData("Target", this);
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                m_poses = m_redPoses;
            }
            if (ally.get() == Alliance.Blue) {
              m_poses = m_bluePoses;
            }
        }
        else {
            System.out.println("No alliance color!");
        }
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("liveTarget", m_ID);
      SmartDashboard.putNumberArray("Field/ReefTarget", getTargetPoseAsDoubles());
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Target");
      builder.addDoubleProperty("TargetID", this::getTargetID, this::setTargetID);
      builder.addDoubleProperty("distance", () -> { return 3.14; }, null);
      builder.addDoubleArrayProperty("Location", this::getTargetPoseAsDoubles, null);
    }
    

}
