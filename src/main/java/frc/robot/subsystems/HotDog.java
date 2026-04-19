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
  private double indexerSpeed = 80;
  private double hotDogSpeed = 30;
  private double reverseHotDogSpeed = -30;
  private double reverseIndexerSpeed = -40;
  private double acceleration = 280;

  private int leftIndexerID = 22;
  private int rightIndexerID = 28;
  private int hotDogID = 13;

  // Talon Classes
  private TalonFX m_LeftIndexer;
  private TalonFX m_RightIndexer;

  private TalonFX m_HotDog;
  private TalonFXConfiguration indexerConfigs;
  private TalonFXConfiguration hotDogConfigs;
  private MotionMagicVelocityVoltage m_velocityRequest;

  private Follower m_Follower;

  

  /** Creates a new Shooter. */
  public HotDog() {
 
    m_LeftIndexer = new TalonFX(leftIndexerID);
    m_RightIndexer = new TalonFX(rightIndexerID);

    m_HotDog = new TalonFX(hotDogID);
    m_velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

    m_Follower = new Follower(leftIndexerID, MotorAlignmentValue.Opposed);

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

    m_LeftIndexer.getConfigurator().apply(indexerConfigs);
    m_RightIndexer.getConfigurator().apply(indexerConfigs);

    m_HotDog.getConfigurator().apply(hotDogConfigs);


  }

  public void setIndexerSpeed(double setpoint)
  {
    SmartDashboard.putNumber("Indexer Commanded Speed", -setpoint);
    m_LeftIndexer.setControl(m_velocityRequest.withVelocity(-setpoint));
    m_RightIndexer.setControl(m_Follower.withLeaderID(leftIndexerID));
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
    m_HotDog.setControl(m_velocityRequest.withVelocity(setpoint));
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
    return m_HotDog.getVelocity().getValueAsDouble();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // shooter.updateTelemetry();

    // Hot Dog Logging
    SmartDashboard.putNumber("HotDog Speed", m_HotDog.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("HotDog Current", m_HotDog.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Left Indexer Speed", m_LeftIndexer.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Indexer Speed", m_RightIndexer.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Indexer Current", m_LeftIndexer.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Right Indexer Current", m_RightIndexer.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // shooter.simIterate();
  }
}
