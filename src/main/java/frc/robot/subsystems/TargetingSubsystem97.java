package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Vision;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class TargetingSubsystem97 extends SubsystemBase {
    private double m_ID = 1;
    private Integer m_aprilTarget = 20;

    public Pose2d[] m_blueBranchPoses = {
      new Pose2d(2,4, Rotation2d.fromDegrees(1)),
      Vision.getAprilTagPose(22, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(22, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(17, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(17, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(18, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(18, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(19, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(19, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(20, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(20, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(21, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(21, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
    };
    
    public Pose2d[] m_blueFeederPoses = {
      Vision.getAprilTagPose(12, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotFeederOffset, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(12, new Transform2d(Constants.robotOffsetCenterToTagCenter, 0, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(12, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotFeederOffset, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(13, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotFeederOffset, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(13, new Transform2d(Constants.robotOffsetCenterToTagCenter, 0, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(13, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotFeederOffset, Rotation2d.fromDegrees(0))),
    };

    public Pose2d[] m_blueParkPoses = {
      Vision.getAprilTagPose(14, new Transform2d(Units.inchesToMeters(39.52 / 2), -1.16, new Rotation2d())),
      Vision.getAprilTagPose(14, new Transform2d(Units.inchesToMeters(39.52 / 2), 0, new Rotation2d())),
      Vision.getAprilTagPose(14, new Transform2d(Units.inchesToMeters(39.52 / 2), 1.16, new Rotation2d())),
    };

    public Pose2d[] m_bluePoses = constructArray(m_blueBranchPoses, m_blueFeederPoses, m_blueParkPoses);

    public Pose2d[] m_redBranchPoses = {
      new Pose2d(1,1,new Rotation2d(1)),
      Vision.getAprilTagPose(9, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(9, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(8, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(8, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(7, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(7, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(6, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(6, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(11, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(11, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(10, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
      Vision.getAprilTagPose(10, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotOffsetRightToTagCenter, Rotation2d.fromDegrees(180))),
    };

    public Pose2d[] m_redFeederPoses = {
      Vision.getAprilTagPose(2, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotFeederOffset, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(2, new Transform2d(Constants.robotOffsetCenterToTagCenter, 0, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(2, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotFeederOffset, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(1, new Transform2d(Constants.robotOffsetCenterToTagCenter, -Constants.robotFeederOffset, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(1, new Transform2d(Constants.robotOffsetCenterToTagCenter, 0, Rotation2d.fromDegrees(0))),
      Vision.getAprilTagPose(1, new Transform2d(Constants.robotOffsetCenterToTagCenter, Constants.robotFeederOffset, Rotation2d.fromDegrees(0))),
    };

    public Pose2d[] m_redParkPoses = {
      Vision.getAprilTagPose(5, new Transform2d(Units.inchesToMeters(-39.52 / 2), 1.16, new Rotation2d())),
      Vision.getAprilTagPose(5, new Transform2d(Units.inchesToMeters(-39.52 / 2), 0, new Rotation2d())), 
      Vision.getAprilTagPose(5, new Transform2d(Units.inchesToMeters(-39.52 / 2), -1.16, new Rotation2d())),
    };
    public Pose2d[] m_redPoses = constructArray(m_redBranchPoses, m_redFeederPoses, m_redParkPoses);

    public Pose2d[] m_poses;

    public Pose2d targetPose = m_bluePoses[1];

    public double getTargetID() { return m_ID; }

    public Integer getTargetAprilTag() { return m_aprilTarget; }

    public void setTargetID(double ID) { 
      Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            m_poses = ally.get() == Alliance.Red ? m_redPoses : m_bluePoses;
            // if (ally.get() == Alliance.Red) {
            //   m_poses = m_redPoses;
            // }
            // if (ally.get() == Alliance.Blue) {
            //   m_poses = m_bluePoses;
            // }
            // else {
            //   m_poses = m_bluePoses;
            //   System.out.println("No alliance color!");
            // }
        }
        else {
          m_poses = m_bluePoses;
          System.out.println("No ally present!");
        }
      m_ID = ID % 21;
      while(m_ID <= 0)
        m_ID += 21;
      SmartDashboard.putNumber("liveTarget2", m_ID);
      targetPose = m_poses[(int)m_ID];
    }

    public void setAprilTargetId(double ID) { 
      m_aprilTarget = (int) (ID % 23);
      while(m_aprilTarget <= 0)
        m_aprilTarget += 22;
      SmartDashboard.putNumber("liveAprilTarget2", m_ID);
    }

    public Pose2d getTargetPose() { return targetPose; }

    public double[] getTargetPoseAsDoubles() { double[] a = {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()}; return a;};

    public TargetingSubsystem97() {
        SmartDashboard.putData("Target", this);
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            m_poses = ally.get() == Alliance.Red ? m_redPoses : m_bluePoses;
            // if (ally.get() == Alliance.Red) {
            //   m_poses = m_redPoses;
            // }
            // if (ally.get() == Alliance.Blue) {
            //   m_poses = m_bluePoses;
            // }
            // else {
            //   m_poses = m_bluePoses;
            //   System.out.println("No alliance color!");
            // }
        }
        else {
          m_poses = m_bluePoses;
          System.out.println("No ally present!");
        }
        targetPose = m_poses[1];
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("liveTarget", m_ID);
      SmartDashboard.putNumber("aprilTarget", m_aprilTarget);
      // SmartDashboard.putNumberArray("Field/TargetedAprilTag", Constants.getPoseAsDoubles(Vision.getAprilTagPose(m_aprilTarget)));
      // SmartDashboard.putNumberArray("Field/AprilOffsetTarget1", Constants.getPoseAsDoubles(Vision.getAprilTagPose(m_aprilTarget, new Transform2d(Units.inchesToMeters(19.5), Units.inchesToMeters(6.5), Rotation2d.fromDegrees(180)))));
      // SmartDashboard.putNumberArray("Field/AprilOffsetTarget2", Constants.getPoseAsDoubles(Constants.get_scoring_tag_offset(Vision.getAprilTagPose(m_aprilTarget, new Transform2d()))));
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
      builder.addDoubleProperty("targetAprilTag", this::getTargetAprilTag, this::setAprilTargetId);
      builder.addDoubleProperty("distance", () -> { return 3.14; }, null);
      builder.addDoubleArrayProperty("Location", this::getTargetPoseAsDoubles, null);
    }
    
    private static Pose2d[] constructArray(Pose2d[] l1, Pose2d[] arr2, Pose2d[] arr3){
              List<Pose2d> list = new ArrayList<Pose2d>();
              list.addAll(Arrays.asList(l1));
              list.addAll(Arrays.asList(arr2));
              list.addAll(Arrays.asList(arr3));
      Pose2d[] array = new Pose2d[list.size()];
      return list.toArray(array);
    }

}
