// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemABS;

public class Arm extends SubsystemABS {

  /** Please look at Arm Drawings for more information on the arm */
    private TalonFX armPivotMotor;
    private CANcoder armPivotEncoder;
    private TalonFX armExtensionMotor;
  public Arm() {}

  @Override
  public void init() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {

  }

  @Override
  public void setDefaultCmd() {

  }

  @Override
  public boolean isHealthy() {
    return false;
  }

  @Override
  public void Failsafe() {

  }
}
