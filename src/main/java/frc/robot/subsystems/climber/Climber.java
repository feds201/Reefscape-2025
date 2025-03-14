// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap.ClimberMap;
import frc.robot.constants.RobotMap.CurrentLimiter;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class Climber extends SubsystemABS {
  /** Creates a new Climber. */
  private CANcoder climberCANCoder;
  private TalonFX climberMotorLeader;
  private TalonFX climberMotorFollower;
  private BooleanSupplier m_climberAtMax;
  public DoubleSupplier m_climberSpeed;
  private DoubleSupplier m_CANcoderValue;
  //private CANcoder climberEncoder;

  public Climber(Subsystems subsystem, String name) {
    super(subsystem, name);
    climberMotorLeader = new TalonFX(ClimberMap.CLIMBER_LEADER_MOTOR);
    climberMotorLeader.getConfigurator()
        .apply(CurrentLimiter.getCurrentLimitConfiguration(ClimberMap.CLIMBER_CURRENT_LIMIT));
    climberMotorFollower = new TalonFX(ClimberMap.CLIMBER_FOLLOWER_MOTOR);
    climberMotorFollower.getConfigurator()
        .apply(CurrentLimiter.getCurrentLimitConfiguration(ClimberMap.CLIMBER_CURRENT_LIMIT));
    climberMotorFollower.setControl(new Follower(climberMotorLeader.getDeviceID(), false));
    climberCANCoder = new CANcoder(ClimberMap.CLIMBER_CANCODER);
    m_climberAtMax = ()-> climberPastMax();
    m_CANcoderValue = ()-> climberCANCoder.getAbsolutePosition().getValueAsDouble();
    //climberEncoder = new CANcoder(ClimberMap.CLIMBER_ENCODER);

    tab.addNumber("Climber CANCoder Position", m_CANcoderValue);

     tab.addBoolean("Climber Out", m_climberAtMax);
        // GenericEntry climberSpeedSetter = tab.add("Climber Speed", 0.0)
        //         .withWidget(BuiltInWidgets.kNumberSlider)
        //         .withProperties(Map.of("min", 0, "max", .2))
        //         .getEntry();
        // m_climberSpeed = () -> climberSpeedSetter.getDouble(0);
  }

  @Override
  public void periodic() {
    m_climberAtMax = ()-> climberPastMax();
    m_CANcoderValue = ()-> climberCANCoder.getAbsolutePosition().getValueAsDouble();
    // This method will be called once per scheduler run
  }

  public void rotateClimber(double speed) {
    climberMotorLeader.set(speed);
    // SmartDashboard.putNumber("Actual Climber Speed", speed);
  }

  public void setPIDTarget(double targetAngle) {
    ClimberMap.climberPID.setSetpoint(targetAngle);
  }

  public double calculatePID(double angle) {
    return ClimberMap.climberPID.calculate(angle);
  }

  public boolean pidAtSetpoint(){
    return ClimberMap.climberPID.atSetpoint();
  }

  public void setPIDTolerance(double tolerance){
    ClimberMap.climberPID.setTolerance(tolerance);
  }

  // public boolean getLeftValue() {
  //   return leftSwitch.get();
  // }

  // public boolean getRightValue() {
  //   return rightSwitch.get();
  // }

  // public boolean climberEngaged(){
  //   return getLeftValue() && getRightValue();
  // }

  public double getEncoderValue() {
    return climberMotorLeader.getPosition().getValueAsDouble();
  }

  public double getCANcoderValue(){
    return climberCANCoder.getAbsolutePosition().getValueAsDouble();
  }

  public void zeroClimber(){
    climberMotorLeader.setPosition(0);
  }

  public boolean climberPastZero(){
    return getCANcoderValue() < -.275 && getCANcoderValue() > -.28;
  }

  public boolean climberPastMax(){
    return getCANcoderValue() > .44 && getCANcoderValue() < .48;
  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void setDefaultCmd() {
    // TODO Auto-generated method stub
  }

  @Override
  public boolean isHealthy() {
    return true;
    // TODO Auto-generated method stub
  }

  @Override
  public void Failsafe() {
  }
}
