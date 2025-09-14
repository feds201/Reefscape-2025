// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lift;
import frc.robot.subsystems.lift.Lift;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RotateElevatorPID extends Command {
  private final Lift c_Elevator;
  private final DoubleSupplier c_ElevatorAngle;
  /* Rotates the Elevator Using the Default PID (infinite). */
  public RotateElevatorPID(Lift elevator, DoubleSupplier elevatorAngle) {
    c_Elevator = elevator;
    c_ElevatorAngle = elevatorAngle;
    addRequirements(c_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    c_Elevator.setPIDTarget(c_ElevatorAngle.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_Elevator.rotateElevatorPID();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_Elevator.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
