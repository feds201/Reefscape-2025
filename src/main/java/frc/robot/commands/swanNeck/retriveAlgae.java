// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.lift.RotateElevatorPID;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;
import frc.robot.subsystems.swanNeck.SwanNeckWheels;

public class retriveAlgae extends SequentialCommandGroup {
  SwanNeck m_SwanNeck;
  SwanNeckWheels m_SwanNeckWheels;
  Lift m_elevator;
  double m_algaePosition;

  /** Command Sequence to Retrieve Algae from the Reef.
   * 
   * @param algaePosition Elevator Position to Retrieve the Algae.
   */
  public retriveAlgae(Lift lift, SwanNeck swanNeck, SwanNeckWheels swanNeckWheels, double algaePosition) {
    m_SwanNeck = swanNeck;
    m_SwanNeckWheels = swanNeckWheels;
    m_elevator = lift;
    m_algaePosition = algaePosition;

    addCommands(
        new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.SAFEANGLE, m_SwanNeck).until(m_SwanNeck::pidAtSetpoint),

        new RotateElevatorPID(m_elevator, () -> m_algaePosition).until(m_elevator::pidAtSetpoint),

        new ParallelCommandGroup(
            new RotateElevatorPID(m_elevator, () -> m_algaePosition),
            new RaiseSwanNeckPID(() -> 0.1170068359375, m_SwanNeck)
        
            ).until(m_SwanNeck::pidAtSetpoint),

        new ParallelDeadlineGroup( 
          new RotateElevatorPID(m_elevator, () -> m_algaePosition),
          new RaiseSwanNeckPID(() -> 0.1170068359375, m_SwanNeck),
          new SpinSwanWheels(m_SwanNeckWheels, () -> IntakeMap.ALGAE_WHEEL_SPEED)
        )
    );
  }
}
