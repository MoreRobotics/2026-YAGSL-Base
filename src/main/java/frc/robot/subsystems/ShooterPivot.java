// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Eyes;

public class ShooterPivot extends SubsystemBase {
  private final Eyes s_Eyes;

  private int shooterPivotID = 16;
  private int canCoderID = 6;
  private double kP = 60;
  private double kI = 0;
  private double kD = 0;
  private double speedkP = 10;
  private double gearRatio = (16384/675)/1.062;
  private double currentLimit = 100;
  private double safePosition = 0.0;
  private double extendedPose = -0.28;
  private double acceleration = 250;
  private double velocity = 50;

  public double homingSpeed = 0.5;
  public double homingCurrentLimit = currentLimit / 2;
  public double homingPosition = 0;

  public double forwardLimit = 0.0;
  public double reverseLimit = -0.278;




  public TalonFX m_ShooterPivot;
  public CANcoder e_ShooterPivot;
  private TalonFXConfiguration configs;
  private MotionMagicVoltage m_Request;
  private CANcoderConfiguration e_Configs;
  private VelocityVoltage m_VelocityRequest;



  /** Creates a new ShooterPivot. */
  public ShooterPivot(Eyes s_Eyes) {
    this.s_Eyes = s_Eyes;
    m_ShooterPivot = new TalonFX(shooterPivotID);
    m_Request = new MotionMagicVoltage(0).withSlot(0);
    e_ShooterPivot = new CANcoder(canCoderID);
    m_VelocityRequest = new VelocityVoltage(0).withSlot(1);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;
    configs.Slot1.kP = speedkP;
    configs.MotionMagic.MotionMagicAcceleration = acceleration;
    configs.MotionMagic.MotionMagicCruiseVelocity = velocity;
    // configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    configs.Feedback.SensorToMechanismRatio = gearRatio;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;




    m_ShooterPivot.getConfigurator().apply(configs);
    m_ShooterPivot.setPosition(0);

  }

  public double getShooterAngle()
  {
    double angle_Rotation;
    if(s_Eyes.getTargetDistance() <= 3.4)
    {
      angle_Rotation = 
      -0.017811*Math.pow(s_Eyes.getTargetDistance(), 2) 
      + 0.015593*s_Eyes.getTargetDistance()
      -0.010468;
    }
    else 
    {
      angle_Rotation = 
      0.0032763*Math.pow(s_Eyes.getTargetDistance(), 4) 
      -0.066184*Math.pow(s_Eyes.getTargetDistance(), 3) 
      + 0.4814171*Math.pow(s_Eyes.getTargetDistance(), 2) 
      -1.5052635*s_Eyes.getTargetDistance()
      +1.5511823;
    }
    return angle_Rotation;
  }

  public void setShooterAngle(double setpoint)
  {

    m_ShooterPivot.setControl(m_Request.withPosition((setpoint)));
     SmartDashboard.putNumber("Shooter Pivot Setpoint", (setpoint));
  }

  public double getShooterPivotSafePose()
  {
    return safePosition;
  }

  public void homeShooter() {
    m_ShooterPivot.setControl(m_VelocityRequest.withVelocity(homingSpeed));
  }

  public void stopSooter()
  {
    m_ShooterPivot.setControl(m_VelocityRequest.withVelocity(0));
  }

  public double getCurrent() {
    return m_ShooterPivot.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter Calculated angle", getShooterAngle());
    SmartDashboard.putNumber("Shooter Pivot Motor Position", m_ShooterPivot.getPosition().getValueAsDouble()); 
    SmartDashboard.putNumber("Shooter Pivot CANCoder Position", e_ShooterPivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Pivot Current", getCurrent());
  }
}
