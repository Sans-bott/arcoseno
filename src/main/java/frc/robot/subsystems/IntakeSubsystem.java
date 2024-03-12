// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_lowerIntakeMotor;
  private final CANSparkMax m_upperIntakeMotor;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    super();
    m_lowerIntakeMotor = new CANSparkMax(IntakeConstants.kLowerIntakeMotorCanId, MotorType.kBrushless);
    m_upperIntakeMotor = new CANSparkMax(IntakeConstants.kUpperIntakeMotorCanId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableIntake()
  {
    m_lowerIntakeMotor.set(IntakeConstants.kLowerIntakeMotorSpeed);
    m_upperIntakeMotor.set(IntakeConstants.kUpperIntakeMotorSpeed);
  }

  public void disableIntake()
  {
    m_lowerIntakeMotor.set(0);
    m_upperIntakeMotor.set(0);
  }
}