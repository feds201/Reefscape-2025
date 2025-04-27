// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.constants.RobotMap.IntakeMap.ReefStops;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseSwanNeckToScoringAngle extends Command {
  /** Creates a new RaiseSwanNeckToScoringAngle. */
  SwanNeck m_SwanNeck;
  Lift m_Lift;
  double setpoint;
  public RaiseSwanNeckToScoringAngle(SwanNeck swanNeck, Lift lift) {
    m_SwanNeck = swanNeck;
    m_Lift = lift;
    addRequirements(m_SwanNeck);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_Lift.getEncoderValueFromMotor() < ElevatorMap.L1ROTATION + 1){
      setpoint = ReefStops.L1ANGLE;
    } else if(m_Lift.getEncoderValueFromMotor() < ElevatorMap.L3ROTATION + 4){
      setpoint = ReefStops.L3ANGLE;
    } else {
      setpoint = ReefStops.L4ANGLE;
    }
    m_SwanNeck.setPIDTarget(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwanNeck.rotateSwanNeckPID();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwanNeck.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_SwanNeck.pidAtSetpoint();
  }
}
