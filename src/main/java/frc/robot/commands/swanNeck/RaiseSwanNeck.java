// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swanNeck.SwanNeck;

public class RaiseSwanNeck extends Command {
  SwanNeck m_swanNeck;
  DoubleSupplier m_speed;
  /** Raise the SwanNeck at a Set Speed. */
  public RaiseSwanNeck(SwanNeck swanNeck, DoubleSupplier speed) {
    m_swanNeck = swanNeck;
    m_speed = speed;
    addRequirements(m_swanNeck);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swanNeck.setPivotSpeed(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swanNeck.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
