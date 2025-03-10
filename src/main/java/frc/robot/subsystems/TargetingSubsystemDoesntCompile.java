package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargetingSubsystemDoesntCompile extends SubsystemBase {

    private long m_ID = 42;

    public long getID() { return m_ID; }
    public void setID(long ID) { m_ID = ID; }
    
    public TargetingSubsystemDoesntCompile() {
        SmartDashboard.putData("Target", this);
    }

    
    @Override
    public void periodic() {
    
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    /*
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Targeting");
      builder.addIntegerProperty("ID", this::getID, this::setID);
      builder.addDoubleProperty("distance", () -> { return 3.14; }, null);
    }
      */
}