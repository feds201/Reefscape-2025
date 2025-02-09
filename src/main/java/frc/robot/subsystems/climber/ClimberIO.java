package frc.robot.subsystems.climber;

public interface ClimberIO{
    public void setRotateVoltage(double voltage);
    public void setPIDTarget(double targetAngle);
    public double calculatePID(double angle);
    public boolean pidAtSetpoint();
    public void setPIDTolerance(double tolerance);
}
