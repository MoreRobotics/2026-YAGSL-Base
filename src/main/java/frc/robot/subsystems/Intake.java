// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Intake Constants
  private double pivotP = 200;//160
  private double pivotI = 0;
  private double pivotD = 0;
  private double pivotAcceleration = 5;//10
  private double pivotVelocity = 1.5;
  private double pivotAccelerationSlow = 2.5;
  private double pivotVelocitySlow = 0.05625;
  private double forwardLimit = .168;
  private double reverseLimit = -.163;
  private double pivotCurrentLimit = 50;
  private double intakeStowPosition = -0.001;
  private double intakeOutPosition = -0.330;//-0.334
  private double intakeMiddlePosition = -0.1525;//-0.165
  private double intakePumpPosition = -0.181;
  private double target = 0;
  private boolean intakeOut = false;
  private double tolerance = 0.005;

  private double rollerP = .25;//.5 too much
  private double rollerI = 0;
  private double rollerD = 0;
  private double rollerV = 0.125;

  private double rollerCurrentLimit = 85;
  private double idleRollerCurrentLimit = 20;
  private double rollerSupplyLimit = 50;
  
  private double intakeSpeed = -70;//60,75
  private double outakeSpeed = 40;
  private double acceleration = 250;

  private double gearRatio = 87.5/1;
  private int intakePivotID = 12;
  private int rightIntakePowerID = 11;
  private int leftIntakePowerID = 29;

  private double intakeVoltage = -9.0;
  private double intakeVoltageIdle = -4.5;

  // Talon Classes
  private TalonFX m_IntakePivot;
  private TalonFX m_RightIntakeRoller;
  private TalonFX m_LeftIntakeRoller;
  private MotionMagicVoltage m_Request;
  private TalonFXConfiguration pivotConfigs;
  private TalonFXConfiguration rollerConfigs;
  private MotionMagicVelocityVoltage m_VelocityRequest;
  private VelocityVoltage m_PivotVelocityRequest;
  private VoltageOut voltageRequest;
  private VoltageOut voltageRequestOutake;
  private VoltageOut voltageRequestStop;
  private VoltageOut voltageRequestIdle;
  private Follower m_RollerFollower;

  /** Creates a new Intake. */
  public Intake() {
    m_IntakePivot = new TalonFX(intakePivotID);
    m_RightIntakeRoller = new TalonFX(rightIntakePowerID);
    m_LeftIntakeRoller = new TalonFX(leftIntakePowerID);
    //e_IntakePivot =  new CANcoder(intakePivotCANCoderID);

    m_Request = new MotionMagicVoltage(0).withSlot(0);
    m_VelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    m_PivotVelocityRequest = new VelocityVoltage(0).withSlot(0);
    
    //Left motor follows the right

    m_RollerFollower = new Follower(rightIntakePowerID, MotorAlignmentValue.Opposed);
    // Motor Config
    pivotConfigs = new TalonFXConfiguration();
    pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfigs.Slot0.kP = pivotP;
    pivotConfigs.Slot0.kI = pivotI;
    pivotConfigs.Slot0.kD = pivotD;
    pivotConfigs.MotionMagic.MotionMagicAcceleration = pivotAcceleration;
    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = pivotVelocity;
    // pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    // pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    pivotConfigs.Feedback.SensorToMechanismRatio = gearRatio;
    pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfigs.CurrentLimits.StatorCurrentLimit = pivotCurrentLimit;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerConfigs = new TalonFXConfiguration();
    rollerConfigs.Slot0.kP = rollerP;
    rollerConfigs.Slot0.kI = rollerI;
    rollerConfigs.Slot0.kD = rollerD;
    rollerConfigs.Slot0.kV = rollerV;
    rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfigs.CurrentLimits.StatorCurrentLimit = rollerCurrentLimit;
    rollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfigs.CurrentLimits.SupplyCurrentLimit = rollerSupplyLimit;
    rollerConfigs.MotionMagic.MotionMagicAcceleration = acceleration;

    m_IntakePivot.getConfigurator().apply(pivotConfigs);
    m_RightIntakeRoller.getConfigurator().apply(rollerConfigs);
    m_LeftIntakeRoller.getConfigurator().apply(rollerConfigs);

    voltageRequest = new VoltageOut(intakeVoltage);
    voltageRequestOutake = new VoltageOut(-intakeVoltage);
    voltageRequestStop = new VoltageOut(0.0);
    voltageRequestIdle = new VoltageOut(intakeVoltageIdle);

    // Set Intake RELATIVE
    m_IntakePivot.setPosition(0);
  }

  public void moveIntake(){
    m_IntakePivot.setControl(m_Request.withPosition(target));
  }

   public void setIntakePivotPosition(double setPosition)
   {
    m_IntakePivot.setPosition(setPosition);
    pivotConfigs.Slot0.kP = pivotP;
    m_IntakePivot.getConfigurator().apply(pivotConfigs);
   }

  public void homeIntakePivot()
  {
    pivotConfigs.Slot0.kP = .25;
    m_IntakePivot.getConfigurator().apply(pivotConfigs);
    m_IntakePivot.setControl(m_PivotVelocityRequest.withVelocity(-3));
  }

  public void stopIntakePivot()
  {
    m_IntakePivot.setControl(m_PivotVelocityRequest.withVelocity(0));
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

  public double getIntakeOutPosition()
  {
    return intakeOutPosition;
  }

  public double getIntakePumpPosition()
  {
    return intakePumpPosition;
  }

  public double getIntakePosition()
  {
    return m_IntakePivot.getPosition().getValueAsDouble();
  }

  public double getIntakePivotCurrent()
  {
    return m_IntakePivot.getStatorCurrent().getValueAsDouble();
  }

  public boolean atPosition()
  {
    return (Math.abs(getIntakePosition() - target) < tolerance);
  }

  public void setSlowMode()
  {
    pivotConfigs.MotionMagic.MotionMagicAcceleration = pivotAccelerationSlow;
    // pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = pivotVelocitySlow;
    m_IntakePivot.getConfigurator().apply(pivotConfigs);
    // intakeOut = true;
  }

  public void setNormalMode()
  {
    pivotConfigs.MotionMagic.MotionMagicAcceleration = pivotAcceleration;
    // pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = pivotVelocity;
    m_IntakePivot.getConfigurator().apply(pivotConfigs);
    // intakeOut = false;
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
    m_RightIntakeRoller.setControl(m_VelocityRequest.withVelocity(speed));
    m_LeftIntakeRoller.setControl(m_RollerFollower);
  }

  public void setIntakeVoltage(){
    m_RightIntakeRoller.setControl(voltageRequest);
    m_LeftIntakeRoller.setControl(m_RollerFollower);
  }
  public void setOutakeVoltage() {
    m_RightIntakeRoller.setControl(voltageRequestOutake);
    m_LeftIntakeRoller.setControl(m_RollerFollower);
  }
  public void setIdleVoltage(){
    m_RightIntakeRoller.setControl(voltageRequestIdle);
    m_LeftIntakeRoller.setControl(m_RollerFollower);
  }
  public void stopIntakeVoltage() {
    m_RightIntakeRoller.setControl(voltageRequestStop);
    m_LeftIntakeRoller.setControl(m_RollerFollower);
  }

  public void setCurrentLimit(double currentLimit)
  {
    rollerConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
     m_RightIntakeRoller.getConfigurator().apply(rollerConfigs);
    m_LeftIntakeRoller.getConfigurator().apply(rollerConfigs);
  }

  public double getIdleRollerCurrentLimit()
  {
    return idleRollerCurrentLimit;
  }

  public double getActiveRollerCurrentLimit()
  {
    return rollerCurrentLimit;
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

    // Intake Logging
    SmartDashboard.putNumber("Intake Pivot Motor Position", getIntakePosition());
    //SmartDashboard.putNumber("Intake Pivot Acceleration Config", pivotConfigs.MotionMagic.MotionMagicAcceleration);
    //SmartDashboard.putNumber("Intake Pivot Velocity Config", m_Request.getPositionMeasure().baseUnitMagnitude());
    SmartDashboard.putNumber("Intake Pivot Target", target);
    // SmartDashboard.putNumber("Intake Pivot Motor Acceleration", m_IntakePivot.getAcceleration().getValueAsDouble());
     SmartDashboard.putNumber("Intake Pivot Motor Velocity", m_IntakePivot.getVelocity().getValueAsDouble());
    //SmartDashboard.putNumber("Intake Pivot CANCoder Position", e_IntakePivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Intake Roller Speed", m_RightIntakeRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Intake Roller Speed", m_LeftIntakeRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Intake Roller Current", m_RightIntakeRoller.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left Intake Roller Current", m_LeftIntakeRoller.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Right Intake Roller Supply Current", m_RightIntakeRoller.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left Intake Roller Supply Current", m_LeftIntakeRoller.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Current", m_IntakePivot.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("Intake Roller Current Limit", rollerConfigs.CurrentLimits.StatorCurrentLimit);

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
