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

  private int leftShooterID = 14;
  private int rightShooterID = 15;
  private double kP = .7;
  private double kI = 0;
  private double kD = 0;
  private double kV = 0.1;
  private double gearRatio = 0;
  private double currentLimit = 70;
  private double acceleration = 250;

  private double kShooter = 1;

  private double shooterSpeed = -57*.85;




  private TalonFX m_LeftShooter;
   private TalonFX m_RightShooter;
  private TalonFXConfiguration configs;
  private MotionMagicVelocityVoltage m_Request;
  private Follower m_Follower;

  /** Creates a new Shooter. */
  public Shooter(Eyes s_Eyes) {
    this.s_Eyes = s_Eyes;

    m_LeftShooter = new TalonFX(leftShooterID);
     m_RightShooter = new TalonFX(rightShooterID);
    m_Request = new MotionMagicVelocityVoltage(0).withSlot(0);
    m_Follower = new Follower(leftShooterID, MotorAlignmentValue.Opposed);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;
    configs.Slot0.kV = kV;
    configs.MotionMagic.MotionMagicAcceleration = acceleration;
    configs.Feedback.SensorToMechanismRatio = gearRatio;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_LeftShooter.getConfigurator().apply(configs);
    m_RightShooter.getConfigurator().apply(configs);

  }

  public void setShooterSpeed(double speed)
  {
    SmartDashboard.putNumber("Shooter Set Speed", speed);
    m_LeftShooter.setControl(m_Request.withVelocity(speed));
    m_RightShooter.setControl(m_Follower.withLeaderID(leftShooterID));
  }

  public double getShooterSpeed()
  {
    double speed;
   
    speed = 
    0.082675*Math.pow(s_Eyes.getTargetDistance(), 5)
    -1.566589*Math.pow(s_Eyes.getTargetDistance(), 4)
    +10.887974*Math.pow(s_Eyes.getTargetDistance(), 3)
    -33.684491*Math.pow(s_Eyes.getTargetDistance(), 2)
    +47.397441*s_Eyes.getTargetDistance()
    +18.379658;


    return -speed*1.05;
    // return shooterSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
    SmartDashboard.putNumber("Left Shooter Motor Speed", m_LeftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Motor Speed", m_RightShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Motor Current", m_LeftShooter.getStatorCurrent().getValueAsDouble());
    
  }
}
