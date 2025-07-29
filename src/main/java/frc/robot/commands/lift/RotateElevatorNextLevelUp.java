// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lift;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.subsystems.lift.Lift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateElevatorNextLevelUp extends Command {
  Lift m_Lift;
  double setpoint;
  /** Creates a new PlaceNextLevelUp. */
  public RotateElevatorNextLevelUp(Lift lift) {
    m_Lift = lift;
    addRequirements(lift);
  }


  @Override
  public void initialize() {
    if(m_Lift.getEncoderValueFromMotor() < ElevatorMap.L2ROTATION + 4){
      setpoint = ElevatorMap.L3ROTATION;
      SmartDashboard.putNumber("elevatorvaluetest", m_Lift.getEncoderValueFromMotor());
    } else if(m_Lift.getEncoderValueFromMotor() < ElevatorMap.L3ROTATION+6){
      setpoint = ElevatorMap.L4ROTATION;
      SmartDashboard.putNumber("elevatorvaluetest", m_Lift.getEncoderValueFromMotor());
    } else {
      setpoint = ElevatorMap.L4ROTATION;
      SmartDashboard.putNumber("elevatorvaluetest", m_Lift.getEncoderValueFromMotor());
    }
    m_Lift.setPIDTarget(setpoint);
  }


  @Override
  public void execute() {
    // SmartDashboard.putNumber("elevatorvaluetest", m_Lift.getEncoderValueFromMotor());

    m_Lift.rotateElevatorPID();
  }

 
  @Override
  public void end(boolean interrupted) {
    m_Lift.setMotorSpeed(0);
  }


  @Override
  public boolean isFinished() {
    return m_Lift.pidAtSetpoint();
  }
}
