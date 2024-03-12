// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AimSubsystem;
import frc.utils.CalculateShooterAngle;

public class AimShooterCommand extends Command {
  AimSubsystem m_aimSubsystem;
  CalculateShooterAngle m_calculateShooterAngle = new CalculateShooterAngle();

  private ShuffleboardTab m_limelightTab;
  private GenericEntry X;
  private GenericEntry Y;
  private GenericEntry Z;

  /** Creates a new AimShooterCommand. */
  public AimShooterCommand(AimSubsystem aimSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_aimSubsystem = aimSubsystem;
    addRequirements(m_aimSubsystem);

    m_limelightTab = Shuffleboard.getTab("Limelight");
    X = m_limelightTab.add("X", 0).getEntry();
    Y = m_limelightTab.add("Y", 0).getEntry();
    Z = m_limelightTab.add("Z", 0).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_aimSubsystem.setSteering(m_calculateShooterAngle.calculateShooterAngle(getPoseXY(m_aimSubsystem.getLimelightPose3D().getX(), m_aimSubsystem.getLimelightPose3D().getY())));
    //m_aimSubsystem.setSteering(20);
    m_aimSubsystem.mover(0.2);
    X.setDouble(m_aimSubsystem.getLimelightPose3D().getX());
    Y.setDouble(m_aimSubsystem.getLimelightPose3D().getY());
    Z.setDouble(m_aimSubsystem.getLimelightPose3D().getZ());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_aimSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getPoseXY(double equis, double lle)
  {
    return Math.sqrt(Math.pow(equis, 2) + Math.pow(lle, 2));
  }

}
