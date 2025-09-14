// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lift;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lift.Lift;

public class RotateElevatorBasic extends Command {
  
  DoubleSupplier m_Speed;
  Lift m_Elevator;
  DoubleSupplier m_volt;

  /* Rotates Elevator At a Constant Speed. */
  public RotateElevatorBasic(DoubleSupplier speed, Lift elevator) {
    m_Speed = speed;
    m_Elevator = elevator;
    addRequirements(elevator);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  @Override
  public void execute(){
    m_Elevator.setMotorSpeed(m_Speed.getAsDouble());
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.setMotorSpeed(0);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
