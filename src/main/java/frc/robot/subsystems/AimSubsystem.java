// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AimingConstants;
import frc.utils.LimelightHelpers;

public class AimSubsystem extends SubsystemBase {
  private final CANSparkMax m_shooterSteeringRight;
  private final CANSparkMax m_shooterSteeringLeft;

  private ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter Data");

  private GenericEntry steeringRightEncoder = shuffleboardTab.add("Steering Right Encoder", 0).getEntry();
  private GenericEntry steeringLeftEncoder = shuffleboardTab.add("Steering Left Encoder", 0).getEntry();

  private final RelativeEncoder m_shooterSteeringRightEncoder;
  private final RelativeEncoder m_shooterSteeringLeftEncoder;

  //Create PID Controller object
  private final SparkPIDController m_shooterSteeringPID;

  //Limelight
  NetworkTable m_limelighNetworkTable;

  /** Creates a new AimSubsystem. */
  public AimSubsystem() {
    m_shooterSteeringLeft = new CANSparkMax(AimingConstants.kShooterSteeringLeftCanId, MotorType.kBrushless);
    m_shooterSteeringRight = new CANSparkMax(AimingConstants.kShooterSteeringRightCanId, MotorType.kBrushless);
    
    m_shooterSteeringRightEncoder = m_shooterSteeringRight.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    m_shooterSteeringLeftEncoder = m_shooterSteeringLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    
    m_shooterSteeringRightEncoder.setPositionConversionFactor(360/80);
    m_shooterSteeringLeftEncoder.setPositionConversionFactor(360/80);
    m_shooterSteeringLeftEncoder.setPosition(0);
    m_shooterSteeringRightEncoder.setPosition(0);
    m_shooterSteeringLeft.setInverted(true);

    m_shooterSteeringPID = m_shooterSteeringRight.getPIDController();
    m_shooterSteeringPID.setP(AimingConstants.kAimingP);
    m_shooterSteeringPID.setI(AimingConstants.kAimingI);
    m_shooterSteeringPID.setD(AimingConstants.kAimingD);

    m_shooterSteeringPID.setPositionPIDWrappingEnabled(true);
    m_shooterSteeringPID.setPositionPIDWrappingMinInput(0);
    m_shooterSteeringPID.setPositionPIDWrappingMaxInput(130);

    m_limelighNetworkTable = NetworkTableInstance.getDefault().getTable(AimingConstants.kLimelightTableName);

    m_shooterSteeringRight.setIdleMode(IdleMode.kBrake);
    m_shooterSteeringLeft.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    steeringRightEncoder.setDouble(m_shooterSteeringRightEncoder.getPosition());
    steeringLeftEncoder.setDouble(m_shooterSteeringLeftEncoder.getPosition());
  }

  public Pose3d getLimelightPose3D()
  {
    return LimelightHelpers.getBotPose3d(AimingConstants.kLimelightTableName);
  }

  public void setSteering(double angle) {
    m_shooterSteeringPID.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void stopMotors()
  {
    m_shooterSteeringLeft.set(0);
    m_shooterSteeringRight.set(0);
  }

  public void mover(double vel) {
    m_shooterSteeringLeft.set(-vel);
    m_shooterSteeringRight.set(vel);
  }
}
