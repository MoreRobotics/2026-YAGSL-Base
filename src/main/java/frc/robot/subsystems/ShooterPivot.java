// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Eyes;

public class ShooterPivot extends SubsystemBase {
  private final Eyes s_Eyes;

  private int shooterPivotID = 10;
  private double kP = 1;
  private double kI = 0;
  private double kD = 0;
  private double forwardLimit = 0;
  private double reverseLimit = 0;
  private double gearRatio = 0;
  private double currentLimit = 70;
  private double safePosition = 0;
  private double acceleration = 50;
  private double velocity = 5;




  private TalonFX m_ShooterPivot;
  private TalonFXConfiguration configs;
  private MotionMagicVoltage m_Request;



  /** Creates a new ShooterPivot. */
  public ShooterPivot(Eyes s_Eyes) {
    this.s_Eyes = s_Eyes;
    m_ShooterPivot = new TalonFX(shooterPivotID);
    m_Request = new MotionMagicVoltage(0).withSlot(0);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;
    configs.MotionMagic.MotionMagicAcceleration = acceleration;
    configs.MotionMagic.MotionMagicCruiseVelocity = velocity;
    // configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    configs.Feedback.SensorToMechanismRatio = gearRatio;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_ShooterPivot.getConfigurator().apply(configs);

  }

  public double getShooterAngle()
  {
    double angle = 70-Math.exp(s_Eyes.getTargetDistance());
    return angle;
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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter Calculated angle", getShooterAngle());
    SmartDashboard.putNumber("Shooter Pivot Motor Position", m_ShooterPivot.getPosition().getValueAsDouble()); 
  }
}
