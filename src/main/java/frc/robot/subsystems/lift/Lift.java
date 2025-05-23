package frc.robot.subsystems.lift;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.RobotMap;
import frc.robot.constants.RobotMap.ElevatorMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class Lift extends SubsystemABS {
    private TalonFX elevatorMotorLeader; // Primary motor
    private TalonFX elevatorMotorFollower; // Follower motor
    // private CANcoder elevatorEncoder; // Range sensor
    private DoubleSupplier m_encoderValue;
    public DoubleSupplier m_elevatorSpeed;
    private BooleanSupplier elevatorAboveThreshold;
    private CANrange frontLeftCanRange;
    private CANrange frontRightCanRange;
    private DigitalInput limitSwitch;

    // private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    private final PIDController pid;
    private final PIDController pidDown;
    private final PIDController pidL3;

    public Lift(Subsystems subsystem, String name) {
        super(subsystem, name);
        elevatorMotorLeader = new TalonFX(RobotMap.ElevatorMap.ELEVATOR_MOTOR);
        elevatorMotorFollower = new TalonFX(RobotMap.ElevatorMap.ELEVATOR_MOTOR2);
        frontLeftCanRange = new CANrange(ElevatorMap.CANRANGE_FL);
        frontRightCanRange = new CANrange(ElevatorMap.CANRANGE_FR);
        limitSwitch = new DigitalInput(9);
       
        // elevatorEncoder.setPosition(0);

        // Configure follower motor
        elevatorMotorFollower.setControl(new Follower(elevatorMotorLeader.getDeviceID(), false));

        // Configure current limits
        elevatorMotorLeader.getConfigurator().apply(
                RobotMap.CurrentLimiter.getCurrentLimitConfiguration(RobotMap.ElevatorMap.ELEVATOR_CURRENT_LIMIT));
        elevatorMotorFollower.getConfigurator().apply(
                RobotMap.CurrentLimiter.getCurrentLimitConfiguration(RobotMap.ElevatorMap.ELEVATOR_CURRENT_LIMIT));

        m_encoderValue = () -> elevatorMotorLeader.getPosition().getValueAsDouble();
        elevatorAboveThreshold = ()-> getElevatorAboveThreshold();
        pid = new PIDController(RobotMap.ElevatorMap.ELEVATOR_P, RobotMap.ElevatorMap.ELEVATOR_I, RobotMap.ElevatorMap.ELEVATOR_D);
        pid.setTolerance(.2);

        pidDown = new PIDController(0.4, 0, 0);
        pidDown.setTolerance(.2);
        pidDown.setSetpoint(1.3);

        pidL3 = new PIDController(0.4, 0, 0);
        pidL3.setTolerance(0.05);
        pidL3.setSetpoint(RobotMap.ElevatorMap.L3ROTATION+3);

        tab.add("Elevator PID", pid)
            .withWidget(BuiltInWidgets.kPIDController);

        tab.add("Elevator PID Down", pidDown).withWidget(BuiltInWidgets.kPIDController);

        tab.addNumber("FL Canrange Val", ()-> frontLeftCanRange.getDistance().getValueAsDouble());
        tab.addNumber("FR Canrange Val", ()-> frontRightCanRange.getDistance().getValueAsDouble());
        tab.addNumber("Elevator Position", m_encoderValue);
        GenericEntry elevatorSpeedSetter = tab.add("Elevator Speed", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", .2))
                .getEntry();
        m_elevatorSpeed = () -> elevatorSpeedSetter.getDouble(0);

        tab.addBoolean("elevator Threshold" , elevatorAboveThreshold);
    }

    @Override
    public void periodic() {
        m_encoderValue = () -> elevatorMotorLeader.getPosition().getValueAsDouble();
        elevatorAboveThreshold = ()-> getElevatorAboveThreshold();

        SmartDashboard.putBoolean("limit switch", elevatorSwitchTriggered());
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void setDefaultCmd() {
    }

    public void setMotorSpeed(double speed) {
        elevatorMotorLeader.set(speed); // Set the speed of the primary motor
       setMotorVolt(speed);
    }

    public void setMotorVolt(double volt){
        elevatorMotorLeader.setControl(new VoltageOut(volt));
    }

    public void setPIDTarget(double target) {
        pid.setSetpoint(target);
    }

    public boolean pidAtSetpoint() {
        return pid.atSetpoint();
    }

    public boolean elevatorSwitchTriggered(){
        return !limitSwitch.get();
    }

    public void rotateElevatorPID() {
        double output = pid.calculate(getEncoderValue());
        if(output > 0){
            output += ElevatorMap.ELEVATOR_S;
        } else {
            output -= ElevatorMap.ELEVATOR_S;
        }
        output += ElevatorMap.ELEVATOR_G;
        setMotorSpeed(output);
    }

    public void rotateElevatorPIDDown(){
        double output = pidDown.calculate(getEncoderValue());
        if(output > 0){
            output += ElevatorMap.ELEVATOR_S;
        } else {
            output -= ElevatorMap.ELEVATOR_S;
        }
        output += ElevatorMap.ELEVATOR_G;
        setMotorSpeed(output);
    }

    public boolean pidDownAtSetpoint(){
        return pidDown.atSetpoint();
    }

    public void rotateElevatorPIDL3(){
        double output = pidL3.calculate(getEncoderValue());
        if(output > 0){
            output += ElevatorMap.ELEVATOR_S;
        } else {
            output -= ElevatorMap.ELEVATOR_S;
        }
        output += ElevatorMap.ELEVATOR_G;
        setMotorSpeed(output);
    }

    public boolean pidL3AtSetpoint(){
        return pidL3.atSetpoint();
    }

    public double getEncoderValue() {
        return m_encoderValue.getAsDouble();
    }

    public double getEncoderValueFromMotor(){
        return elevatorMotorLeader.getPosition().getValueAsDouble();
    }

    public double getElevatorHeight() {
        return m_encoderValue.getAsDouble() * ElevatorMap.ELEVATOR_CIRCUMFERENCE;
    }

    

    public void zeroElevator(){
        elevatorMotorLeader.setPosition(0);
    }

    public void setElevatorAtLimitHeight(){
        elevatorMotorLeader.setPosition(.45);
    }

    public boolean getElevatorAboveThreshold(){
        return getEncoderValue() > RobotMap.ElevatorMap.L3ROTATION + 1;
    }

    public boolean getElevatorAtBelowL4(){
        return getEncoderValue() < RobotMap.ElevatorMap.L4ROTATION;
    }

    public boolean getElevatorAtBarge(){
        return getEncoderValue() > RobotMap.ElevatorMap.BARGEROTATION-1;
    }

    public double getFLDistance(){
        return frontLeftCanRange.getDistance().getValueAsDouble();
    }

    public double getFRDistance(){
        return frontRightCanRange.getDistance().getValueAsDouble();
    }

    public double getFRStdDev(){
        return frontRightCanRange.getDistanceStdDev().getValueAsDouble();
    }

    public double getFLStdDev(){
        return frontLeftCanRange.getDistanceStdDev().getValueAsDouble();
    }

    public boolean getRobotInFrontOfCoral(){
        return ((getFLDistance() > .19) || (getFRDistance() >.19)) && (getFRStdDev() < 0.05) && (getFLStdDev() < 0.05);
    }

    public boolean getRobotNotInFrontOfCoral(){
        return !(((getFLDistance() > .19) || (getFRDistance() >.19)) && (getFRStdDev() < 0.05) && (getFLStdDev() < 0.05));
    }

    @Override
    public boolean isHealthy() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void Failsafe() {
        // elevatorMotorLeader.disable();
        // elevatorMotorFollower.disable();
    }

    public void StopElevator(){
        elevatorMotorLeader.set(0);

    }

    // public void configurePIDFProfile(){
    //     elevatorMotorLeader.getConfigurator().apply(RobotMap.ElevatorMap.ELEVATOR_PIDF);
    // }
}
