// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import java.lang.annotation.ElementType;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.lift.RotateElevatorPID;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceLTwo extends SequentialCommandGroup {
  SwanNeck m_SwanNeck;
  Lift m_elevator;
  /** Creates a new scoreLTwo. */
  public PlaceLTwo(Lift lift, SwanNeck swanNeck) {
    m_SwanNeck = swanNeck;
    m_elevator = lift;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RaiseSwanNeckPID(()-> IntakeMap.ReefStops.SAFEANGLE , m_SwanNeck).until(m_SwanNeck :: pidAtSetpoint), 

    new RotateElevatorPID(m_elevator, ()-> ElevatorMap.L2ROTATION).until(m_elevator :: pidAtSetpoint), 

    new ParallelCommandGroup(new RotateElevatorPID(m_elevator, ()-> ElevatorMap.L2ROTATION), 
    new RaiseSwanNeckPID(()-> IntakeMap.ReefStops.L2ANGLE, m_SwanNeck)).until(m_SwanNeck :: pidAtSetpoint),

    new ParallelDeadlineGroup(new WaitCommand(1), new RotateElevatorPID(m_elevator, ()-> ElevatorMap.L2ROTATION), 
    new SpinSwanWheels(m_SwanNeck, ()-> IntakeMap.WHEEL_SPEED_SCORE)),
    new RaiseSwanNeckPID(()-> IntakeMap.ReefStops.SAFEANGLE, m_SwanNeck).until(m_SwanNeck :: pidAtSetpoint)
     );
  }
}
