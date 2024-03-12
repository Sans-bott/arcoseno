// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.MotorRowBehaviour;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_entranceLeftMotor;
  private final CANSparkMax m_entranceRightMotor;
  private final CANSparkMax m_backLeftMotor;
  private final CANSparkMax m_backRightMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_entranceLeftMotor = new CANSparkMax(ShooterConstants.kEntranceLeftMotorCanId, MotorType.kBrushless);
    m_entranceRightMotor = new CANSparkMax(ShooterConstants.kEntranceRightMotorCanId, MotorType.kBrushless);
    m_backLeftMotor = new CANSparkMax(ShooterConstants.kBackLeftMotorCanId, MotorType.kBrushless);
    m_backRightMotor = new CANSparkMax(ShooterConstants.kBackRightMotorCanId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {}

  public void setFrontShooterRow(double speed, ShooterConstants.MotorRowBehaviour behaviour) {
    if (behaviour == MotorRowBehaviour.INTAKE) {
      m_entranceLeftMotor.set(-speed);
      m_entranceRightMotor.set(speed);
    } else if (behaviour == MotorRowBehaviour.OUTTAKE) {
      m_entranceLeftMotor.set(speed);
      m_entranceRightMotor.set(-speed);
    }
  }

  public void setBackShooterRow(double speed, ShooterConstants.MotorRowBehaviour behaviour) {
    if (behaviour == MotorRowBehaviour.INTAKE) {
      m_backLeftMotor.set(-speed);
      m_backRightMotor.set(speed);
    } else if (behaviour == MotorRowBehaviour.OUTTAKE) {
      m_backLeftMotor.set(speed);
      m_backRightMotor.set(-speed);
    }
  }

  public void stopAllMotors() {
    m_entranceLeftMotor.set(0);
    m_entranceRightMotor.set(0);
    m_backLeftMotor.set(0);
    m_backRightMotor.set(0);
  }

  public double getShooterAngle(double encoderValue) {
    return encoderValue * 90 / 23;
  }
}
