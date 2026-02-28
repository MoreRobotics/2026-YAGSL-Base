// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private double pivotP = 60;
  private double pivotI = 0;
  private double pivotD = 0;
  private double pivotAcceleration = 20.0;
  private double pivotVelocity = 2;
  private double pivotAccelerationSlow = 0.28125;
  private double pivotVelocitySlow = 0.05625;
  private double forwardLimit = .168;
  private double reverseLimit = -.163;
  private double pivotCurrentLimit = 100;
  private double intakeStowPosition = -.162;
  private double intakeOutPosition = .167;
  private double intakeMiddlePosition = .002;
  private double target = 0;
  private boolean intakeOut = false;
  private double tolerance = 0.03;

  private double rollerP = .25;//.5 too much
  private double rollerI = 0;
  private double rollerD = 0;
  private double rollerCurrentLimit = 100;
  private double intakeSpeed = 60;
  private double outakeSpeed = -60;

  private double gearRatio = 87.5/1;
  private int intakePivotID = 12;
  private int intakeRollerID = 11;
  private int intakePivotCANCoderID = 12;

  private TalonFX m_IntakePivot;
  private TalonFX m_IntakeRoller;
  private CANcoder e_IntakePivot;
  private MotionMagicVoltage m_Request;
  private TalonFXConfiguration pivotConfigs;
  private TalonFXConfiguration rollerConfigs;
  private VelocityVoltage m_VelocityRequest;

  /** Creates a new Intake. */
  public Intake() {
    m_IntakePivot = new TalonFX(intakePivotID);
    m_IntakeRoller = new TalonFX(intakeRollerID);
    e_IntakePivot =  new CANcoder(intakePivotCANCoderID);

    m_Request = new MotionMagicVoltage(0).withSlot(0);
    m_VelocityRequest = new VelocityVoltage(0).withSlot(0);

    pivotConfigs = new TalonFXConfiguration();
    pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfigs.Slot0.kP = pivotP;
    pivotConfigs.Slot0.kI = pivotI;
    pivotConfigs.Slot0.kD = pivotD;
    pivotConfigs.MotionMagic.MotionMagicAcceleration = pivotAcceleration;
    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = pivotVelocity;
    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    pivotConfigs.Feedback.SensorToMechanismRatio = gearRatio;
    pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfigs.CurrentLimits.StatorCurrentLimit = pivotCurrentLimit;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerConfigs = new TalonFXConfiguration();
    rollerConfigs.Slot0.kP = rollerP;
    rollerConfigs.Slot0.kI = rollerI;
    rollerConfigs.Slot0.kD = rollerD;
    rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfigs.CurrentLimits.StatorCurrentLimit = rollerCurrentLimit;


    m_IntakePivot.getConfigurator().apply(pivotConfigs);
    m_IntakeRoller.getConfigurator().apply(rollerConfigs);
    m_IntakePivot.setPosition(e_IntakePivot.getPosition().getValueAsDouble());
    
    
  }

  public void setIntakePosition(){
    m_IntakePivot.setControl(m_Request.withPosition(target));
  }

  public void changeTarget(){
    if(intakeOut)
    {
      target = intakeStowPosition;
    }
    else
    {
      target = intakeOutPosition;
    }
  }

  public void changeState()
  {
    intakeOut = !intakeOut;
  }


  public void setIntakeTarget(double setpoint)
  {
    target = setpoint;
  }

  public double getIntakeMiddlePosition()
  {
    return intakeMiddlePosition;
  }

  public double getIntakePosition()
  {
    return m_IntakePivot.getPosition().getValueAsDouble();
  }

  public boolean atPosition()
  {
    return (Math.abs(getIntakePosition() - target) < tolerance);
  }

  public void setSlowMode()
  {
    pivotConfigs.MotionMagic.MotionMagicAcceleration = pivotAccelerationSlow;
    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = pivotVelocitySlow;
    m_IntakePivot.getConfigurator().apply(pivotConfigs);
    intakeOut = true;
  }

  public void setNormalMode()
  {
    pivotConfigs.MotionMagic.MotionMagicAcceleration = pivotAcceleration;
    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = pivotVelocity;
    m_IntakePivot.getConfigurator().apply(pivotConfigs);
    intakeOut = false;
  }

  public void setTarget(double newTarget)
  {
    target = newTarget;
  }

  public void setState(boolean intakeOut)
  {
    this.intakeOut = intakeOut;
  }
  

  public void setIntakeSpeed(double speed){
    SmartDashboard.putNumber("Intake Roller Commanded Speed", speed);
    m_IntakeRoller.setControl(m_VelocityRequest.withVelocity(speed));
  }
  

  public double getIntakeSpeed()
  {
    return intakeSpeed;
  }

  public double getOutakeSpeed()
  {
    return outakeSpeed;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pivot Motor Position", getIntakePosition());
    SmartDashboard.putNumber("Intake Pivot Acceleration Config", pivotConfigs.MotionMagic.MotionMagicAcceleration);
    SmartDashboard.putNumber("Intake Pivot Velocity Config", m_Request.getPositionMeasure().baseUnitMagnitude());
    SmartDashboard.putNumber("Intake Pivot Target", target);
    SmartDashboard.putNumber("Intake Pivot Motor Acceleration", m_IntakePivot.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Motor Velocity", m_IntakePivot.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot CANCoder Position", e_IntakePivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Roller Speed", m_IntakeRoller.getVelocity().getValueAsDouble());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
