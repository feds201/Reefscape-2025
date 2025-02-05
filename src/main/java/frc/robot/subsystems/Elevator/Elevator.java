package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemABS {
    private final TalonFX climoter13 = new TalonFX(13); // Primary motor
    private final TalonFX climoter15 = new TalonFX(15); // Follower motor
    private final CANrange measure = new CANrange(17); // Range sensor
    private final ShuffleboardTab tab = Shuffleboard.getTab("CANrange Status");

    public Elevator() {
        // Configure follower motor
        climoter15.setControl(new Follower(climoter13.getDeviceID(), false));

        // Add Shuffleboard widget for the range sensor
        tab.add("Elevator Position", getRangePosition());
    }

    public void setMotorSpeed(double speed) {
        climoter13.set(speed); // Set the speed of the primary motor
    }

    public double getRangePosition() {
        return measure.getDistance().getValueAsDouble(); // Get the range position
    }

    @Override
    public void periodic() {
        // Update Shuffleboard with the latest range position
        tab.add("Elevator Position", getRangePosition());
    }
}
