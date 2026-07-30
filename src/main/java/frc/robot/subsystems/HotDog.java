// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;


public class HotDog extends SubsystemBase {

  // Hot Dog Contants
  private double indexerP = .25;
  private double indexerI = 0;
  private double indexerD = 0;
  private double indexerV = .16;
  private double hotDogP = .3;//.35
  private double hotDogI = 0;
  private double hotDogD = 0;
  private double hotDogV = .13;//.15

  private double indexerCurrentLimit = 80;
  private double hotDogCurrentLimit = 80;

  private double indexerSupplyLimit = 50;
  private double hotDogSupplyLimit = 50;

  private double indexerSpeed = 80;//80, 60
  private double hotDogSpeed = 60;//can go to 60
  private double reverseHotDogSpeed = -30;
  private double reverseIndexerSpeed = -40;

  private int IndexerID = 22;
  private int hotDogRightID = 28;
  private int hotDogLeftID = 13;

  private double indexerVoltage = 12.0;
  private double hotDogVoltage = 12.0;

  // Talon Classes
  private TalonFX m_Indexer;


  private TalonFX m_HotDogLeft;
  private TalonFX m_HotDogRight;

  private Follower m_Follower;

  private VoltageOut indexerVoltageRequest;
  private VoltageOut indexerVoltageRequestOutake;
  private VoltageOut indexerVoltageRequestStop;

  private VoltageOut hotDogVoltageRequest;
  private VoltageOut hotDogVoltageRequestOutake;
  private VoltageOut hotDogVoltageRequestStop;

  private boolean indexerRunning = false;

  // YAMS mechanisms - indexer and leader (left) hotdog roller, each wrapping their own TalonFX.
  // hotDogRight stays a plain raw TalonFX, untouched by YAMS: it's a Follower in the velocity
  // control path (setHotDogSpeed) but gets independent direct voltage in the voltage-control
  // methods below - that mixed pattern is preserved exactly as it was.
  private FlyWheel m_IndexerFlyWheel;
  private SmartMotorController m_IndexerMotor;
  private FlyWheel m_HotDogFlyWheel;
  private SmartMotorController m_HotDogMotor;

  /** Creates a new Shooter. */
  public HotDog() {

    m_Indexer = new TalonFX(IndexerID);

    m_HotDogLeft = new TalonFX(hotDogLeftID);
    m_HotDogRight = new TalonFX(hotDogRightID);

    m_Follower = new Follower(hotDogLeftID, MotorAlignmentValue.Opposed);

    // Motor Configs
    SmartMotorControllerConfig indexerMotorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(indexerP, indexerI, indexerD)
        .withFeedforward(new SimpleMotorFeedforward(0, indexerV, 0))
        // No SensorToMechanismRatio was ever set in the original config, meaning the real robot
        // already runs 1:1 - not a placeholder here.
        .withGearing(MechanismGearing.kOne)
        .withStatorCurrentLimit(Amps.of(indexerCurrentLimit))
        .withSupplyCurrentLimit(Amps.of(indexerSupplyLimit))
        .withIdleMode(MotorMode.COAST)
        // Placeholder indexer wheel mass for simulation physics only - not a measured value.
        .withMomentOfInertia(Inches.of(1.5), Kilograms.of(0.3))
        .withTelemetry("Indexer", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    m_IndexerMotor = new TalonFXWrapper(m_Indexer, DCMotor.getKrakenX60(1), indexerMotorConfig);
    // Placeholder indexer wheel diameter for simulation physics only - not a measured value.
    m_IndexerFlyWheel = new FlyWheel(new FlyWheelConfig().withDiameter(Inches.of(2)), m_IndexerMotor);

    SmartMotorControllerConfig hotDogMotorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(hotDogP, hotDogI, hotDogD)
        .withFeedforward(new SimpleMotorFeedforward(0, hotDogV, 0))
        .withGearing(MechanismGearing.kOne)
        .withStatorCurrentLimit(Amps.of(hotDogCurrentLimit))
        .withSupplyCurrentLimit(Amps.of(hotDogSupplyLimit))
        .withIdleMode(MotorMode.COAST)
        // Placeholder hotdog wheel mass for simulation physics only - not a measured value.
        .withMomentOfInertia(Inches.of(1.5), Kilograms.of(0.3))
        .withTelemetry("HotDog", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    m_HotDogMotor = new TalonFXWrapper(m_HotDogLeft, DCMotor.getKrakenX60(1), hotDogMotorConfig);
    // Placeholder hotdog wheel diameter for simulation physics only - not a measured value.
    m_HotDogFlyWheel = new FlyWheel(new FlyWheelConfig().withDiameter(Inches.of(2)), m_HotDogMotor);

    indexerVoltageRequest = new VoltageOut(-indexerVoltage);
    indexerVoltageRequestOutake = new VoltageOut(indexerVoltage);
    indexerVoltageRequestStop = new VoltageOut(0.0);

    hotDogVoltageRequest = new VoltageOut(hotDogVoltage);
    hotDogVoltageRequestOutake = new VoltageOut(-hotDogVoltage);
    hotDogVoltageRequestStop = new VoltageOut(0.0);

  }

  public void setIndexerSpeed(double setpoint)
  {
    SmartDashboard.putNumber("Indexer Commanded Speed", -setpoint);
    m_IndexerMotor.setVelocity(RotationsPerSecond.of(-setpoint));
    indexerRunning = setpoint != 0;
  }

  /**
   * Whether the indexer is currently commanded to feed a piece toward the shooter. Used in
   * simulation as the trigger moment for launching a held Fuel piece as a projectile.
   */
  public boolean isIndexerRunning()
  {
    return indexerRunning;
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
    m_HotDogMotor.setVelocity(RotationsPerSecond.of(setpoint));
    m_HotDogRight.setControl(m_Follower.withLeaderID(hotDogLeftID));
  }

  public double getHotDogSpeed()
  {

    return hotDogSpeed;
  }

  public double getReverseHotDogSpeed()
  {
    return reverseHotDogSpeed;
  }

  public double getHotDogMotorSpeed()
  {
    return m_HotDogLeft.getVelocity().getValueAsDouble();
  }

  public void setIndexerVoltage(){
    m_IndexerMotor.setVoltage(Volts.of(-indexerVoltage));
  }
  public void setIndexerVoltageOutake(){
    m_IndexerMotor.setVoltage(Volts.of(indexerVoltage));
  }
  public void stopIndexerVoltage() {
    m_IndexerMotor.setVoltage(Volts.of(0.0));
  }

  public void setHotDogVoltage(){
    m_HotDogMotor.setVoltage(Volts.of(hotDogVoltage));
    m_HotDogRight.setControl(hotDogVoltageRequestOutake);
  }
  public void setHotDogVoltageOutake(){
    m_HotDogMotor.setVoltage(Volts.of(-hotDogVoltage));
    m_HotDogRight.setControl(hotDogVoltageRequest);
  }
  public void stopHotDogVoltage() {
    m_HotDogMotor.setVoltage(Volts.of(0.0));
    m_HotDogRight.setControl(hotDogVoltageRequestStop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_IndexerFlyWheel.updateTelemetry();
    m_HotDogFlyWheel.updateTelemetry();

    // Hot Dog Logging
    SmartDashboard.putNumber("Right HotDog Speed", m_HotDogRight.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right HotDog Current", m_HotDogRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left HotDog Speed", m_HotDogLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left HotDog Current", m_HotDogLeft.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Left Indexer Speed", m_Indexer.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Indexer Current", m_Indexer.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    m_IndexerFlyWheel.simIterate();
    m_HotDogFlyWheel.simIterate();
  }
}
