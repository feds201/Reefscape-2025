// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lift;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.subsystems.lift.Lift;

public class RotateElevatorNextLevelDown extends Command {
  Lift m_Lift;
  boolean useNormalPid = false;
  double setpoint;
  final double ELEVATOR_POSITION_TOLERANCE = 2;
  /** Rotate the elevator to the next level downwards from current elevator level (l4, l3, etc.) */
  public RotateElevatorNextLevelDown(Lift lift) {
    m_Lift = lift;
    addRequirements(lift);
  }


  @Override
  public void initialize() {
    useNormalPid = false;
    double encoderValue = m_Lift.getEncoderValueFromMotor();
    if(encoderValue > ElevatorMap.L4ROTATION + ELEVATOR_POSITION_TOLERANCE){

      setpoint = ElevatorMap.L4ROTATION;
    }else if(encoderValue > ElevatorMap.L4ROTATION - ELEVATOR_POSITION_TOLERANCE){

      setpoint = ElevatorMap.L3ROTATION;
    } else if(encoderValue > ElevatorMap.L3ROTATION - ELEVATOR_POSITION_TOLERANCE){

      setpoint = ElevatorMap.L2ROTATION;
    } else if(encoderValue > ElevatorMap.L2ROTATION - ELEVATOR_POSITION_TOLERANCE) {

      setpoint = ElevatorMap.L1ROTATION;
    } else if(encoderValue > ElevatorMap.L1ROTATION - ELEVATOR_POSITION_TOLERANCE){
      // Small deviation from the pattern, used to allow easy access to level 2 from intaking or 0 position.
      setpoint = ElevatorMap.L2ROTATION;
      useNormalPid = true;
    } else {

      setpoint = ElevatorMap.CORAL_INTAKE_ROTATION;
    }
    if(useNormalPid){
      m_Lift.setPIDTarget(setpoint);
    } else {
      m_Lift.setPIDSafeTarget(setpoint);
    }
  }


  @Override
  public void execute() {
    if(useNormalPid){
      m_Lift.rotateElevatorPID();
    } else {
      m_Lift.rotateElevatorPIDSafe();
    }
  }

 
  @Override
  public void end(boolean interrupted) {
    m_Lift.setMotorSpeed(0);
  }


  @Override
  public boolean isFinished() {
    boolean finished = false;
    if(useNormalPid){
      finished = m_Lift.pidAtSetpoint();
    } else {
      finished =  m_Lift.pidSafeAtSetpoint();
    }
    return finished;
  }
}
