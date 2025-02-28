package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class pythonicCurve {
  private final double m_accelRateLimit;      // Maximum acceleration rate
  private final double m_decelRateLimit;      // Maximum deceleration rate
  private final double m_accelSmoothing;      // Smoothness for acceleration
  private final double m_decelSmoothing;      // Smoothness for deceleration
  
  private double m_prevVal;
  private double m_prevTime;

  public pythonicCurve(double accelRateLimit, double decelRateLimit, double accelSmoothing, double decelSmoothing, double initialValue) {
    m_accelRateLimit = accelRateLimit;
    m_decelRateLimit = decelRateLimit;
    m_accelSmoothing = accelSmoothing;
    m_decelSmoothing = decelSmoothing;
    m_prevVal = initialValue;
    m_prevTime = MathSharedStore.getTimestamp();
  }

  public pythonicCurve(double rateLimit, double smoothingFactor) {
    this(rateLimit, rateLimit, smoothingFactor, smoothingFactor, 0);
  }

  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - m_prevTime;
    
    // Determine whether we are accelerating or decelerating
    if (input > m_prevVal) {
      // Acceleration
      double accelLimit = m_accelRateLimit * m_accelSmoothing;
      m_prevVal += MathUtil.clamp(input - m_prevVal, 0, accelLimit * elapsedTime);
    } else if (input < m_prevVal) {
      // Deceleration
      double decelLimit = m_decelRateLimit * m_decelSmoothing;
      m_prevVal += MathUtil.clamp(input - m_prevVal, -decelLimit * elapsedTime, 0);
    }

    m_prevTime = currentTime;
    return m_prevVal;
  }

  private double lastValue() {
    return m_prevVal;
  }

  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = MathSharedStore.getTimestamp();
  }
}
