// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Eyes;

public class ShooterPivot extends SubsystemBase {
  private final Eyes s_Eyes;

  private int shooterPivotID = 16;
  private int canCoderID = 6;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double forwardLimit = 0;
  private double reverseLimit = 0;
  private double gearRatio = 67.232/1;
  private double currentLimit = 70;
  private double safePosition = 0;
  private double acceleration = 0;
  private double velocity = 0;




  private TalonFX m_ShooterPivot;
  private TalonFXConfiguration configs;
  private MotionMagicVoltage m_Request;
  private CANcoder e_ShooterPivot;
  private CANcoderConfiguration e_Configs;



  /** Creates a new ShooterPivot. */
  public ShooterPivot(Eyes s_Eyes) {
    this.s_Eyes = s_Eyes;
    m_ShooterPivot = new TalonFX(shooterPivotID);
    m_Request = new MotionMagicVoltage(0).withSlot(0);
    e_ShooterPivot = new CANcoder(canCoderID);

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
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;



    m_ShooterPivot.getConfigurator().apply(configs);
    m_ShooterPivot.setPosition(e_ShooterPivot.getPosition().getValueAsDouble()/2.8125);

  }

  public double getShooterAngle()
  {
    double angle = 2*(12*(Math.pow(s_Eyes.getTargetDistance(), 2))-95.2*s_Eyes.getTargetDistance()+232.9)/360;//divide by 360 to get rotations
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
    SmartDashboard.putNumber("Shooter Pivot CANCoder Position", e_ShooterPivot.getPosition().getValueAsDouble()/2.8125);
  }
}
