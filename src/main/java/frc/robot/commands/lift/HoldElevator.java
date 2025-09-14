// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lift.Lift;

public class HoldElevator extends Command {
  Lift m_Lift;
  /** Holds the Elevator at its' Current Height (infinite). */
  public HoldElevator(Lift lift) {
    m_Lift = lift;
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Lift.setPIDTarget(m_Lift.getEncoderValueFromMotor());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Lift.rotateElevatorPID();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Lift.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
