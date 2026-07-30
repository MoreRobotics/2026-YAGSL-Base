// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Eyes;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterPivot extends SubsystemBase {
  private final Eyes s_Eyes;

  // Shooter Pivot Constants
  private int shooterPivotID = 26;
  private double kP = 340;//135 error = 0.003//160 good
  private double kI = 0;
  private double kD = 0;
  private double kV = 0;//2.8
  private double speedkP = 10;
  // NOTE: this is integer division (15500/180 = 86, not 86.111) in the original code - preserved
  // exactly rather than silently "fixed", since it's not part of this migration's scope.
  private double gearRatio = 15500/180;
  private double currentLimit = 45;//35
  private double safePosition = -0.004;
  private double acceleration = 125;//250
  private double velocity = 50;

  // Homing Command Constants
  public double homingSpeed = 0.5;
  public double homingCurrentLimit = currentLimit / 2;
  public double homingPosition = 0.001;

  // Pivot Limits
  public double forwardLimit = 0.0;
  public double reverseLimit = -0.0727;



  // Talon + Cancoder Classes
  public TalonFX m_ShooterPivot;
  private CANcoderConfiguration e_Configs;
  private VelocityVoltage m_VelocityRequest;

  // YAMS mechanism wrapping the same TalonFX, for setAngle/getAngle-driven position control and
  // simulated arm physics. The separate raw-TalonFX Slot1 velocity homing routine below
  // (homeShooter/stopSooter) stays untouched by YAMS - see plan notes on scoping this migration to
  // the core position-control path only.
  private Arm m_Arm;
  private SmartMotorController m_PivotMotor;



  /** Creates a new ShooterPivot. */
  public ShooterPivot(Eyes s_Eyes) {
    this.s_Eyes = s_Eyes;
    m_ShooterPivot = new TalonFX(shooterPivotID);
    m_VelocityRequest = new VelocityVoltage(0).withSlot(1);

    SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(kP, kI, kD)
        .withFeedforward(new ArmFeedforward(0, 0, kV, 0))
        .withGearing(new MechanismGearing(gearRatio))
        // Matches the m_ShooterPivot.setPosition(0) relative-zero at the end of this constructor.
        .withStartingPosition(Rotations.of(0))
        .withTrapezoidalProfile(RotationsPerSecond.of(velocity), RotationsPerSecondPerSecond.of(acceleration))
        .withStatorCurrentLimit(Amps.of(currentLimit))
        .withIdleMode(MotorMode.BRAKE)
        // Placeholder arm mass for simulation physics only - not a measured value, tune once the
        // real shooter pivot's mass is known.
        .withMomentOfInertia(Inches.of(12), Kilograms.of(3))
        .withTelemetry("ShooterPivot", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    m_PivotMotor = new TalonFXWrapper(m_ShooterPivot, DCMotor.getKrakenX60(1), motorConfig);
    // Only the reverse soft limit was enabled in the original config (forward is commented out
    // there too) - preserved exactly via the wrapper's single-sided limit setter.
    m_PivotMotor.setMechanismLowerLimit(Rotations.of(reverseLimit));
    m_PivotMotor.setMechanismLimitsEnabled(true);

    // Slot1 (raw velocity control for the current-based homing routine below) isn't part of the
    // YAMS Arm - applied separately via CTRE's targeted Slot1Configs overload so it survives
    // regardless of what YAMS's own config application touches.
    Slot1Configs slot1 = new Slot1Configs();
    slot1.kP = speedkP;
    m_ShooterPivot.getConfigurator().apply(slot1);

    // Placeholder arm length for simulation physics only - not a measured value.
    ArmConfig armConfig = new ArmConfig()
        .withLength(Inches.of(18))
        .withHardLimits(Rotations.of(reverseLimit), Rotations.of(forwardLimit));
    m_Arm = new Arm(armConfig, m_PivotMotor);

    // Set Shooter RELATIVE
    m_ShooterPivot.setPosition(0);

  }


  // Shooter Angle Calculation
  public double getShooterAngle()
  {
    double angle_Rotation;
    angle_Rotation =
    -0.00091667*Math.pow(s_Eyes.getTargetDistance(), 4)
    +0.01425*Math.pow(s_Eyes.getTargetDistance(), 3)
    -0.07805208*Math.pow(s_Eyes.getTargetDistance(), 2)
    +0.15907812*s_Eyes.getTargetDistance()
    -0.11112598;

    if(angle_Rotation <= -0.071)
    {
      return -0.071;
    }
    else if(angle_Rotation >= -0.004)
    {
      return -0.004;
    }
    else{
      return angle_Rotation;
    }

  }

  public void setShooterAngle(double setpoint)
  {
    m_PivotMotor.setPosition(Rotations.of(setpoint));
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
    m_Arm.updateTelemetry();

    // Shooter Pivot Logging
    SmartDashboard.putNumber("Shooter Calculated angle", getShooterAngle());
    SmartDashboard.putNumber("Shooter Pivot Motor Position", m_ShooterPivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Pivot Current", getCurrent());
  }

  @Override
  public void simulationPeriodic() {
    m_Arm.simIterate();
  }
}
