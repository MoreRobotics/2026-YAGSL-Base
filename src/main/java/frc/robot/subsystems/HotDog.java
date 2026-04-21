// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HotDog extends SubsystemBase {

  // Hot Dog Contants
  private double indexerP = .25;
  private double indexerI = 0;
  private double indexerD = 0;
  private double indexerV = .12;
  private double hotDogP = .35;//.35
  private double hotDogI = 0;
  private double hotDogD = 0;
  private double hotDogV = .15;//.15
  
  private double currentLimit = 40;
  private double hotDogCurrentLimit = 70;
  private double indexerSpeed = 80 * 1.052;
  private double hotDogSpeed = 30;
  private double reverseHotDogSpeed = -30;
  private double reverseIndexerSpeed = -40;
  private double acceleration = 280;

  private int IndexerID = 22;
  private int hotDogRightID = 28;
  private int hotDogLeftID = 13;

  // Talon Classes
  private TalonFX m_Indexer;


  private TalonFX m_HotDogLeft;
  private TalonFX m_HotDogRight;
  private TalonFXConfiguration indexerConfigs;
  private TalonFXConfiguration hotDogConfigs;
  private MotionMagicVelocityVoltage m_velocityRequest;

  private Follower m_Follower;

  

  /** Creates a new Shooter. */
  public HotDog() {
 
    m_Indexer = new TalonFX(IndexerID);

    m_HotDogLeft = new TalonFX(hotDogLeftID);
    m_HotDogRight = new TalonFX(hotDogRightID);
    m_velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

    m_Follower = new Follower(hotDogLeftID, MotorAlignmentValue.Opposed);

    // Motor Configs
    indexerConfigs = new TalonFXConfiguration();
    indexerConfigs.Slot0.kP = indexerP;
    indexerConfigs.Slot0.kI = indexerI;
    indexerConfigs.Slot0.kD = indexerD;
    indexerConfigs.Slot0.kV = indexerV;
    indexerConfigs.MotionMagic.MotionMagicAcceleration = acceleration;
    indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;

    hotDogConfigs = new TalonFXConfiguration();
    hotDogConfigs.Slot0.kP = hotDogP;
    hotDogConfigs.Slot0.kI = hotDogI;
    hotDogConfigs.Slot0.kD = hotDogD;
    hotDogConfigs.Slot0.kV = hotDogV;
    hotDogConfigs.MotionMagic.MotionMagicAcceleration = acceleration;
    hotDogConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    hotDogConfigs.CurrentLimits.StatorCurrentLimit = hotDogCurrentLimit;

    m_Indexer.getConfigurator().apply(indexerConfigs);

    m_HotDogLeft.getConfigurator().apply(hotDogConfigs);
    m_HotDogRight.getConfigurator().apply(hotDogConfigs);


  }

  public void setIndexerSpeed(double setpoint)
  {
    SmartDashboard.putNumber("Indexer Commanded Speed", -setpoint);
    m_Indexer.setControl(m_velocityRequest.withVelocity(-setpoint));
  }

  public double getIndexerSpeed()
  {
    return indexerSpeed;
  }

  public double getReverseIndexerSpeed()
  {
    return reverseIndexerSpeed;
  }

  public void setHotDogSpeed(double setpoint)
  {
    SmartDashboard.putNumber("HotDog Commanded Speed", setpoint);
    m_HotDogLeft.setControl(m_velocityRequest.withVelocity(setpoint));
    m_HotDogRight.setControl(m_Follower.withLeaderID(hotDogLeftID));
  }

  public double getHotDogSpeed()
  {
    
    return hotDogSpeed;
  }

  public double getReverseHotDogSpeed()
  {
    return reverseHotDogSpeed;
  }

  public double getHotDogMotorSpeed()
  {
    return m_HotDogLeft.getVelocity().getValueAsDouble();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // shooter.updateTelemetry();

    // Hot Dog Logging
    SmartDashboard.putNumber("Right HotDog Speed", m_HotDogRight.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right HotDog Current", m_HotDogRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left HotDog Speed", m_HotDogLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left HotDog Current", m_HotDogLeft.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Left Indexer Speed", m_Indexer.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Indexer Current", m_Indexer.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // shooter.simIterate();
  }
}
