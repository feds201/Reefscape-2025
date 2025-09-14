// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swanNeck;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.lift.RotateElevatorPID;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;
import frc.robot.subsystems.swanNeck.SwanNeckWheels;

public class PlaceBarge extends SequentialCommandGroup {
  SwanNeck m_SwanNeck;
  SwanNeckWheels m_SwanNeckWheels;
  Lift m_elevator;
  /** Barge scoring command sequence */
  public PlaceBarge(Lift lift, SwanNeck swanNeck, SwanNeckWheels swanNeckWheels) {
    m_SwanNeck = swanNeck;
    m_SwanNeckWheels = swanNeckWheels;
    m_elevator = lift;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     new ParallelCommandGroup(new SpinSwanWheels(swanNeckWheels, ()-> IntakeMap.ALGAE_WHEEL_SPEED), new SequentialCommandGroup(
    new RaiseSwanNeckPIDAlgae(()-> IntakeMap.ReefStops.BARGEANGLE, m_SwanNeck).until(m_SwanNeck :: pidAtSetpoint), 
    new ParallelDeadlineGroup(new RotateElevatorPID(m_elevator, ()-> ElevatorMap.BARGEROTATION).until(m_elevator :: pidAtSetpoint), new RaiseSwanNeckPIDAlgae(()-> IntakeMap.ReefStops.BARGEANGLE, m_SwanNeck)), 
    new ParallelCommandGroup(new RotateElevatorPID(m_elevator, ()-> ElevatorMap.BARGEROTATION), new RaiseSwanNeckPIDAlgae(()-> IntakeMap.ReefStops.BARGEANGLE, m_SwanNeck)))) 
   
    // ,

    // new ParallelRaceGroup(new WaitCommand(0.35), new MoveBack(DrivetrainConstants.drivetrain)),
    // new RotateElevatorSafePID(m_elevator).until(m_elevator :: pidL3AtSetpoint)
    );
  }
}
