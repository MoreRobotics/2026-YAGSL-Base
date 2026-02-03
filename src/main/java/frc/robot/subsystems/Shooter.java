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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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


public class Shooter extends SubsystemBase {

  private double shooterP = .1;
  private double shooterI = 0;
  private double shooterD = 0;
  private double hotDogP = .1;
  private double hotDogI = 0;
  private double hotDogD = 0;
  
  private double currentLimit = 0;
  private double shooterSpeed = 20;
  private double hotDogSpeed = 20;
  private double reverseHotDogSpeed = -20;

  private int shooterID = 13;
  private int hotDogID = 0;
  private double gearRatio = 0;

  private TalonFX m_Shooter;
  private TalonFX m_HotDog;
  private TalonFXConfiguration shooterConfigs;
  private TalonFXConfiguration hotDogConfigs;
  private VelocityVoltage m_velocityRequest;


  

  /** Creates a new Shooter. */
  public Shooter() {
    m_Shooter = new TalonFX(shooterID);
    m_HotDog = new TalonFX(hotDogID);
    m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    shooterConfigs = new TalonFXConfiguration();
    shooterConfigs.Slot0.kP = shooterP;
    shooterConfigs.Slot0.kI = shooterI;
    shooterConfigs.Slot0.kD = shooterD;
    shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;

    hotDogConfigs = new TalonFXConfiguration();
    hotDogConfigs.Slot0.kP = hotDogP;
    hotDogConfigs.Slot0.kI = hotDogI;
    hotDogConfigs.Slot0.kI = hotDogI;
    hotDogConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    hotDogConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;

    m_Shooter.getConfigurator().apply(shooterConfigs);
    m_HotDog.getConfigurator().apply(hotDogConfigs);


  }

  public void setShooterSpeed(double setpoint)
  {
    m_Shooter.setControl(m_velocityRequest.withVelocity(setpoint));
  }

  public double getShootSpeed()
  {
    return shooterSpeed;
  }

  public void setHotDogSpeed(double setpoint)
  {
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // shooter.simIterate();
  }
}
