// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.lift.RotateElevatorPID;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;
import frc.robot.subsystems.swanNeck.SwanNeckWheels;

public class PlaceLFourAuton extends SequentialCommandGroup {
  SwanNeck m_SwanNeck;
  SwanNeckWheels m_SwanNeckWheels;
  Lift m_elevator;

  /** L4 Command Sequence Without the Concluding SwanNeck safety command*/
  public PlaceLFourAuton(Lift lift, SwanNeck swanNeck, SwanNeckWheels swanNeckWheels) {
    m_SwanNeck = swanNeck;
    m_SwanNeckWheels = swanNeckWheels;
    m_elevator = lift;
    addCommands(

        new RotateElevatorPID(m_elevator, () -> ElevatorMap.L4ROTATION).until(m_elevator::pidAtSetpoint),

        new ParallelCommandGroup(
            new RotateElevatorPID(m_elevator, () -> ElevatorMap.L4ROTATION),
            new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.L4ANGLE, m_SwanNeck)).until(m_SwanNeck::pidAtSetpoint),

        new ParallelDeadlineGroup(
            new WaitCommand(.2),
            new RotateElevatorPID(m_elevator, () -> ElevatorMap.L4ROTATION),
            new SpinSwanWheels(m_SwanNeckWheels, () -> IntakeMap.WHEEL_SPEED_SCORE * 2))
    );
  }
}
