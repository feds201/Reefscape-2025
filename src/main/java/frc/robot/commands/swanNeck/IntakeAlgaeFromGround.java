// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.lift.RotateElevatorPID;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;
import frc.robot.subsystems.swanNeck.SwanNeckWheels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgaeFromGround extends SequentialCommandGroup {
  private SwanNeck m_SwanNeck;
  private Lift m_Lift;
  private SwanNeckWheels m_SwanNeckWheels;
  /** Creates a new IntakeAlgaeFromGround. */
  public IntakeAlgaeFromGround(SwanNeck swanNeck, Lift lift, SwanNeckWheels swanNeckWheels) {
    m_Lift = lift;
    m_SwanNeck = swanNeck;
    m_SwanNeckWheels = swanNeckWheels;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SequentialCommandGroup(
      new RotateElevatorPID(m_Lift, ()-> 1.7).until(m_Lift :: pidAtSetpoint),
      new SequentialCommandGroup(
        new ParallelCommandGroup(
        new RotateElevatorPID(m_Lift, ()-> 1.7),
        new RaiseSwanNeckPID(()-> 0.25, m_SwanNeck),
        new SpinSwanWheels(m_SwanNeckWheels, ()-> IntakeMap.ALGAE_WHEEL_SPEED)
      ))
    ));
  }
}
