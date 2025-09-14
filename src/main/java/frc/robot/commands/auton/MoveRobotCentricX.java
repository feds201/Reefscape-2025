// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class MoveRobotCentricX extends Command {
  CommandSwerveDrivetrain m_swerve;
  DoubleSupplier m_speed;
   /** Moves robot-centrically backwards (infinite) */
  public MoveRobotCentricX(CommandSwerveDrivetrain swerve, DoubleSupplier speed) {
    m_swerve = swerve;
    m_speed = speed;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.setControl(new SwerveRequest.RobotCentric().withVelocityX(m_speed.getAsDouble()).withVelocityY(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.setControl(new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
