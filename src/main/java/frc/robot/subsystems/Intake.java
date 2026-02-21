// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase {
  private double pivotP = 60;
  private double pivotI = 0;
  private double pivotD = 0;
  private double pivotAcceleration = 20.0;
  private double pivotVelocity = 2;
  private double pivotAccelerationSlow = 1.0;
  private double pivotVelocitySlow = 0.1;
  private double forwardLimit = .168;
  private double reverseLimit = -.163;
  private double pivotCurrentLimit = 100;
  private double intakeStowPosition = -.162;
  private double intakeOutPosition = .167;
  private double target = 0;
  private boolean intakeOut = false;
  private double tolerance = 0.03;

  private double rollerP = .25;//.5 too much
  private double rollerI = 0;
  private double rollerD = 0;
  private double rollerCurrentLimit = 100;
  private double intakeSpeed = 75;
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
