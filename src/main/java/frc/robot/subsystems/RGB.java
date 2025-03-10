package frc.robot.subsystems;

import java.util.Random;

//import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class RGB extends SubsystemBase {
    private AddressableLED m_led;
    public AddressableLEDBuffer LEDs;

    private static int m_LEDLoopCount;

    private final static int kTotalLEDs = 100;
    private static Random m_random = new Random();
    private final static int kIntroTime = 12;

    public RGB(int port) {
        m_led = new AddressableLED(port);

        // Reuse buffer
        // Default to a length of 100, start empty output
        // Length is expensive to set, so only set it once, then just update data
        LEDs = new AddressableLEDBuffer(kTotalLEDs);
        m_led.setLength(LEDs.getLength());

        // Set the data
        m_led.setData(LEDs);
        m_led.start();
    }

    public void clearLEDs(int start, int end) {
        for (int i = start; i <= end; i++)
            LEDs.setRGB(i, 0, 0, 0);
    }

    public void clearLEDs() {
        clearLEDs(4, LEDs.getLength() - 1);
    }

    // 35 for outer only, 59 for inner and outer
    //public final int kNumLEDsInRing = 59;

    @Override
    public void periodic() {
        m_LEDLoopCount++;
        if (m_LEDLoopCount < 5)
            return;
        m_LEDLoopCount = 0;

        double fpgaTime = Timer.getFPGATimestamp();
        int fpgaTimeInt = (int) fpgaTime;

        if (fpgaTimeInt < kIntroTime) {
            DoIntroAnimation2(fpgaTime);
            return;
        }
        // clearLEDs();
        
        double kLoopSpeed = 7;

        double matchTime = Timer.getMatchTime();

        double loopTime = fpgaTime * kLoopSpeed;
        @SuppressWarnings("unused")
        int loopTimeInt = (int) loopTime;

        final int NINE_CIRCLE_START = 4;
        final int NINE_CIRCLE_END = NINE_CIRCLE_START + 20 - 1;
        final int NINE_STROKE_START = NINE_CIRCLE_END + 1;
        final int NINE_STROKE_END = NINE_STROKE_START + 9 - 1;
        final int SEVEN_TOP_START = NINE_STROKE_END + 1;
        final int SEVEN_TOP_END = SEVEN_TOP_START + 10 - 1;
        final int SEVEN_STROKE_START = SEVEN_TOP_END + 1;
        final int SEVEN_STROKE_END = SEVEN_STROKE_START + 18 - 1;
        
        // // 9 circle
        // for (int j = NINE_CIRCLE_START; j <= NINE_CIRCLE_END; j++) {
        //     LEDs.setRGB(j, 100, 0, 0);
        // }
        // // 9 stroke
        // for (int j = NINE_STROKE_START; j <= NINE_STROKE_END; j++) {
        //     LEDs.setRGB(j, 100, 100, 0);
        // }
        // // 7 top
        // for (int j = SEVEN_TOP_START; j <= SEVEN_TOP_END; j++) {
        //     LEDs.setRGB(j, 0, 0, 100);
        // }
        // // 7 stroke
        // for (int j = SEVEN_STROKE_START; j <= SEVEN_STROKE_END; j++) {
        //     LEDs.setRGB(j, 0, 100, 0);
        // }

        // Full Rainbow with wipe in
        for (int j = NINE_CIRCLE_START; j <= SEVEN_STROKE_END; j++) {
            if (j >= (fpgaTime - kIntroTime) * 20)
                break;
            LEDs.setHSV(j, (int) (j * 2.95), 255, 80);
        }

        @SuppressWarnings("unused")
        int kNumLEDsInTotal = SEVEN_STROKE_END + 1;


        // if CORAL is in the intake
        if (RobotContainer.coralInIntake()) {
            LEDs.setRGB(0, 0, 128+(int)fpgaTime%128, 0);
        } else {
            LEDs.setRGB(0, 255, 0, 0);
        }

        SmartDashboard.putNumber("Match Time", matchTime);
        SmartDashboard.putNumber("FPGA Time", fpgaTime);
    
        // This method will be called once per scheduler run
        m_led.setData(LEDs);
    }

    @SuppressWarnings("unused")
    private void DoIntroAnimation1(double fpgaTime) {
        int LED = 0;
        
        for (int j = 0; j < 10; j++) {
            LED = m_random.nextInt(kTotalLEDs);
            LEDs.setHSV(LED, m_random.nextInt(200), m_random.nextInt(20) + 220, m_random.nextInt((int) fpgaTime * 10 + j * 2));
            LED = m_random.nextInt(kTotalLEDs);
            int greyValue = (int) (fpgaTime * 1) + j;
            LEDs.setRGB(LED, greyValue, greyValue, greyValue);
        }
        LED = m_random.nextInt(kTotalLEDs);
        LEDs.setRGB(LED, 200, 200, 200);
        LED = m_random.nextInt(kTotalLEDs);
        LEDs.setRGB(LED, 0, 0, 0);
        


        SmartDashboard.putNumber("FPGA Time", fpgaTime);
    
        // This method will be called once per scheduler run
        m_led.setData(LEDs);
    }

    private void DoIntroAnimation2(double fpgaTime) {
        int LED = 0;
        
        if (fpgaTime < 7)
            clearLEDs();
        for (int j = 0; j < (int) fpgaTime + 1; j++) {
            LED = m_random.nextInt(kTotalLEDs);
            LEDs.setHSV(LED, m_random.nextInt(180), m_random.nextInt(20) + 220, m_random.nextInt(50 + 200));
            LED = m_random.nextInt(kTotalLEDs);
            int greyValue = 255;
            LEDs.setRGB(LED, greyValue, greyValue, greyValue);
        }

        SmartDashboard.putNumber("FPGA Time", fpgaTime);
    
        // This method will be called once per scheduler run
        m_led.setData(LEDs);
    }

    
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}

