// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;


public class HotDog extends SubsystemBase {

  private double indexerP = .33;
  private double indexerI = 0;
  private double indexerD = 0;
  private double hotDogP = .33;
  private double hotDogI = 0;
  private double hotDogD = 0;
  private double hotDogV = .15;
  
  private double currentLimit = 100;
  private double indexerSpeed = 85*.5;
  private double hotDogSpeed = 85;
  private double reverseHotDogSpeed = -40;
  private double reverseIndexerSpeed = -40;
  private double acceleration = 500;

  private int indexerID = 13;
  private int hotDogID = 0;
  private double gearRatio = 0;

   private TalonFX m_Indexer;
  private TalonFX m_HotDog;
  private TalonFXConfiguration indexerConfigs;
  private TalonFXConfiguration hotDogConfigs;
  private MotionMagicVelocityVoltage m_velocityRequest;


  

  /** Creates a new Shooter. */
  public HotDog() {
     m_Indexer = new TalonFX(indexerID);
    m_HotDog = new TalonFX(hotDogID);
    m_velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

    indexerConfigs = new TalonFXConfiguration();
    indexerConfigs.Slot0.kP = indexerP;
    indexerConfigs.Slot0.kI = indexerI;
    indexerConfigs.Slot0.kD = indexerD;
    indexerConfigs.MotionMagic.MotionMagicAcceleration = acceleration;
    indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;

    hotDogConfigs = new TalonFXConfiguration();
    hotDogConfigs.Slot0.kP = hotDogP;
    hotDogConfigs.Slot0.kI = hotDogI;
    hotDogConfigs.Slot0.kD = hotDogD;
    hotDogConfigs.Slot0.kV = hotDogV;
    hotDogConfigs.MotionMagic.MotionMagicAcceleration = acceleration;
    hotDogConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    hotDogConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;

     m_Indexer.getConfigurator().apply(indexerConfigs);
    m_HotDog.getConfigurator().apply(hotDogConfigs);


  }

  public void setIndexerSpeed(double setpoint)
  {
    m_Indexer.setControl(m_velocityRequest.withVelocity(setpoint));
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




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // shooter.updateTelemetry();
    SmartDashboard.putNumber("HotDog Speed", m_HotDog.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("HotDog Current", m_HotDog.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // shooter.simIterate();
  }
}
