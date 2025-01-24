package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class retrieveAlgae extends Command {
  private AlgaeCoralIntake intakeSubsystem; // NUll
  private boolean isFinished;
  private PIDcontroller pidController;

  public retrieveAlgae(AlgaeCoralIntake intake) {
  this.intakeSubsystem = intake;

  }

  
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
