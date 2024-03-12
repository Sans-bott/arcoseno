// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.sequencies;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.AccelerateShooterCommand;
import frc.robot.Commands.RetainShooterCommand;
import frc.robot.Commands.ShootShooterCommand;
import frc.robot.Commands.StopShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequenceCommand extends SequentialCommandGroup {
  private ShooterSubsystem m_shooterSubsystem;

  /** Creates a new ShootCommand. */
  public ShootSequenceCommand(ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RetainShooterCommand(shooterSubsystem).withTimeout(0.3),
                new AccelerateShooterCommand(m_shooterSubsystem).withTimeout(2), 
                new ShootShooterCommand(m_shooterSubsystem).withTimeout(1.5), 
                new StopShooterCommand(m_shooterSubsystem));
  }
}
