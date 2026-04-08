// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Eyes;

public class Shooter extends SubsystemBase {
  private final Eyes s_Eyes;

  // Shooter Constants
  private int topLeftShooterID = 23;
  private int topRightShooterID = 25;

  private int bottomLeftShooterID = 14;
  private int bottomRightShooterID = 24;

  private double kP = .85 / 2;
  private double kI = 0;
  private double kD = 0;
  private double kV = 0.1;
  private double gearRatio = 0;
  private double currentLimit = 40;
  private double acceleration = 80;

  private double kShooter = 1;
  private double shooterSpeed = -57*.6;



  // Motor Classes
  private TalonFX m_TopLeftShooter;
  private TalonFX m_TopRightShooter;

  private TalonFX m_BottomLeftShooter;
  private TalonFX m_BottomRightShooter;

  private TalonFXConfiguration configs;
  private MotionMagicVelocityVoltage m_Request;

  private Follower m_LeftFollower;
  private Follower m_RightFollower;

  /** Creates a new Shooter. */
  public Shooter(Eyes s_Eyes) {
    this.s_Eyes = s_Eyes;

    m_TopLeftShooter = new TalonFX(topLeftShooterID);
    m_TopRightShooter = new TalonFX(topRightShooterID);
    m_BottomLeftShooter = new TalonFX(bottomLeftShooterID);
    m_BottomRightShooter = new TalonFX(bottomRightShooterID);


    m_Request = new MotionMagicVelocityVoltage(0).withSlot(0);

    m_LeftFollower = new Follower(bottomLeftShooterID, MotorAlignmentValue.Aligned);
    m_RightFollower = new Follower(bottomRightShooterID, MotorAlignmentValue.Aligned);

    // Motor Configs
    configs = new TalonFXConfiguration();
    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;
    configs.Slot0.kV = kV;
    configs.MotionMagic.MotionMagicAcceleration = acceleration;
    configs.Feedback.SensorToMechanismRatio = gearRatio;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_TopLeftShooter.getConfigurator().apply(configs);
    m_TopRightShooter.getConfigurator().apply(configs);

    m_BottomLeftShooter.getConfigurator().apply(configs);
    m_BottomRightShooter.getConfigurator().apply(configs);

  }

  public void setShooterSpeed(double speed)
  {
    SmartDashboard.putNumber("Shooter Set Speed", speed);
    m_TopLeftShooter.setControl(m_Request.withVelocity(-speed));
    m_BottomLeftShooter.setControl(m_LeftFollower.withLeaderID(topLeftShooterID));

    m_TopRightShooter.setControl(m_Request.withVelocity(speed));
    m_BottomRightShooter.setControl(m_RightFollower.withLeaderID(topRightShooterID));
  }

  public void setShooterVoltage(double voltage)
  {
    m_TopLeftShooter.setVoltage(voltage);
    m_BottomLeftShooter.setControl(m_LeftFollower.withLeaderID(topLeftShooterID));

    m_TopRightShooter.setVoltage(-voltage);
    m_BottomRightShooter.setControl(m_RightFollower.withLeaderID(topRightShooterID));
  }

  public double getCurrentSpeed() {
    return m_TopLeftShooter.getVelocity().getValueAsDouble();
  }

  public double getShooterSpeed()
  {
    double speed;
   
    speed = 
    2.393631*s_Eyes.getTargetDistance()
    +31.625928;
    if(speed > 48.165)
    {
      return -48.165;
    }
    else if(speed < 34.2)
    {
      return -34.2;
    }
    else{
      return -speed;
    }

    // return shooterSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Logging Variables
    SmartDashboard.putNumber("Top Left Shooter Motor Speed", m_TopLeftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Left Shooter Motor Speed", m_BottomLeftShooter.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Top Right Shooter Motor Speed", m_TopRightShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Right Shooter Motor Speed", m_BottomRightShooter.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Top Left Shooter Motor Current", m_TopLeftShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Left Shooter Motor Current", m_BottomLeftShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Top Right Shooter Motor Current", m_TopRightShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Right Shooter Motor Current", m_BottomRightShooter.getStatorCurrent().getValueAsDouble());
    
  }
}
