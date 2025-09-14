// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.lift.RotateElevatorDownPID;
import frc.robot.commands.lift.RotateElevatorPID;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;
import frc.robot.subsystems.swanNeck.SwanNeckWheels;

public class PlaceLThree extends SequentialCommandGroup {
  SwanNeck m_SwanNeck;
  SwanNeckWheels m_SwanNeckWheels;
  Lift m_elevator;
  /** L3 Scoring Command Sequence */
  public PlaceLThree(Lift lift, SwanNeck swanNeck, SwanNeckWheels swanNeckWheels) {
    m_SwanNeck = swanNeck;
    m_SwanNeckWheels = swanNeckWheels;
    m_elevator = lift;

    addCommands(new RaiseSwanNeckPID(()-> IntakeMap.ReefStops.AGAINSTREEFANGLE , m_SwanNeck).until(m_SwanNeck :: pidAtSetpoint), 

    new RotateElevatorPID(m_elevator, ()-> ElevatorMap.L3ROTATION).until(m_elevator :: pidAtSetpoint), 

    new ParallelCommandGroup(new RotateElevatorPID(m_elevator, ()-> ElevatorMap.L3ROTATION), 
    new RaiseSwanNeckPID(()-> IntakeMap.ReefStops.SAFEANGLE, m_SwanNeck)).until(m_SwanNeck :: pidAtSetpoint),

    new ParallelDeadlineGroup(new WaitCommand(.35), new RotateElevatorPID(m_elevator, ()-> ElevatorMap.L3ROTATION), 
    new SpinSwanWheels(m_SwanNeckWheels, ()-> IntakeMap.WHEEL_SPEED_SCORE)),
    new RaiseSwanNeckPID(()-> IntakeMap.ReefStops.SAFEANGLE, m_SwanNeck).until(m_SwanNeck :: pidAtSetpoint),
    new RotateElevatorDownPID(m_elevator).until(m_elevator :: pidDownAtSetpoint)
    
     );
  }
}
