package frc.robot.subsystems.climber;

public class ClimberIORobot implements ClimberIO {
    private Climber climber;

    public ClimberIORobot(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void setRotateVoltage(double voltage) {
        climber.setRotateVoltage(voltage);
    }

    @Override
    public void setPIDTarget(double targetAngle) {
        climber.setPIDTarget(targetAngle);
    }

    @Override
    public double calculatePID(double angle) {
        return climber.calculatePID(angle);
    }

    @Override
    public boolean pidAtSetpoint() {
        return climber.pidAtSetpoint();
    }

    @Override
    public void setPIDTolerance(double tolerance) {
        climber.setPIDTolerance(tolerance);
    }
    
}
