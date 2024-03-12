// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AccelerateShooterCommand extends Command {

  private ShooterSubsystem m_shooterSubsystem;


  /** Creates a new AccelerateIntakeCommand. */
  public AccelerateShooterCommand(ShooterSubsystem shooterSubsystem) {
  m_shooterSubsystem = shooterSubsystem;
  addRequirements(m_shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFrontShooterRow(ShooterConstants.kMaxVelocity, ShooterConstants.MotorRowBehaviour.OUTTAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
