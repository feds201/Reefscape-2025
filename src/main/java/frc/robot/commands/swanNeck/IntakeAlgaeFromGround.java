// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.lift.RotateElevatorPID;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;
import frc.robot.subsystems.swanNeck.SwanNeckWheels;

public class IntakeAlgaeFromGround extends SequentialCommandGroup {
  private SwanNeck m_SwanNeck;
  private Lift m_Lift;
  private SwanNeckWheels m_SwanNeckWheels;
  /** Command Sequence to Intake Ground Algae. */
  public IntakeAlgaeFromGround(SwanNeck swanNeck, Lift lift, SwanNeckWheels swanNeckWheels) {
    m_Lift = lift;
    m_SwanNeck = swanNeck;
    m_SwanNeckWheels = swanNeckWheels;

    addCommands(new SequentialCommandGroup(
      new RotateElevatorPID(m_Lift, ()-> 1.67).until(m_Lift :: pidAtSetpoint),
      new SequentialCommandGroup(
        new ParallelCommandGroup(
        new RotateElevatorPID(m_Lift, ()-> 1.67),
        new RaiseSwanNeckPID(()-> 0.22, m_SwanNeck) //0.25
        ,
        new SpinSwanWheels(m_SwanNeckWheels, ()-> IntakeMap.ALGAE_WHEEL_SPEED)
      ))
    ));
  }
}
