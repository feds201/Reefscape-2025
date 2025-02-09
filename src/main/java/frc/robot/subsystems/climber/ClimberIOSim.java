package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {

    // Simulation state fields
    private double simulatedAngle = 0;
    private double targetAngle = 0;
    private double tolerance = 0.1;  // default tolerance

    @Override
    public void setRotateVoltage(double voltage) {
        // Simple simulation: update angle based on voltage with a scaling factor.
        simulatedAngle += voltage * 0.1; // scale factor for simulation update
    }

    @Override
    public void setPIDTarget(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    @Override
    public double calculatePID(double angle) {
        // Simple proportional controller simulation
        double kP = 0.1;  // simulation proportional gain 
        double error = targetAngle - angle;
        return kP * error;
    }

    @Override
    public boolean pidAtSetpoint() {
        return Math.abs(targetAngle - simulatedAngle) <= tolerance;
    }

    @Override
    public void setPIDTolerance(double tolerance) {
        this.tolerance = tolerance;
    }
}
