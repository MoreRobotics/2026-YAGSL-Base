// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

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
  private double intakeOutPosition = -0.333;//-0.330
  private double intakeBlockPosition = -0.079;//-0.165
  private double intakePumpPosition = -0.196;//-0.165
  private double target = 0;
  private boolean intakeOut = false;
  private double tolerance = 0.005;

  private double rollerP = .25;//.5 too much
  private double rollerI = 0;
  private double rollerD = 0;
  private double rollerV = 0.125;
  private double rollerCurrentLimit = 80;
  private double idleRollerCurrentLimit = 20;
  private double rollerSupplyLimit = 50;

  private double intakeSpeed = -70;//60,75
  private double outakeSpeed = 40;

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
  private Follower m_RollerFollower;

  private boolean intaking = false;

  // YAMS mechanisms - pivot (Arm) and right/leader roller (FlyWheel), both wrapping the same
  // TalonFX objects. Left roller stays a plain CTRE Follower, untouched by YAMS.
  private Arm m_PivotArm;
  private SmartMotorController m_PivotMotor;
  private FlyWheel m_RollerFlyWheel;
  private SmartMotorController m_RollerMotor;

  /**
   * Publishes the intake pivot's live pose for AdvantageScope's articulated CAD component
   * (Robot_MoreRobotics2026/model_0.glb - the intake is the moving component in that export, the
   * shooter hood is static/part of model.glb). Rotation axis assumes YAMS's default Arm movement
   * plane (vertical/XZ, i.e. pitch about Y) - needs visual confirmation/calibration against the
   * actual CAD export, alongside config.json's zeroedPosition/zeroedRotations.
   */
  private final StructArrayPublisher<Pose3d> intakeComponentPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Telemetry/ComponentPoses3d", Pose3d.struct).publish();

  /** Creates a new Intake. */
  public Intake() {
    m_IntakePivot = new TalonFX(intakePivotID);
    m_RightIntakeRoller = new TalonFX(rightIntakePowerID);
    m_LeftIntakeRoller = new TalonFX(leftIntakePowerID);

    //Left motor follows the right
    m_RollerFollower = new Follower(leftIntakePowerID, MotorAlignmentValue.Opposed);

    // NOTE: neither forward nor reverse soft limit is actually enabled on the real robot today
    // (both were already commented out in the original config) - preserved exactly, these limits
    // are only used below for the Arm's simulated range of motion, not real software enforcement.
    SmartMotorControllerConfig pivotMotorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(pivotP, pivotI, pivotD)
        // NOTE: withSimClosedLoopController does NOT apply here - confirmed by reading
        // TalonFXWrapper's actual source: setPosition() always routes through CTRE's native
        // MotionMagic request via Phoenix6's own simulated TalonFX firmware (using the REAL
        // pivotP gain above), for both real hardware and simulation. A sim-only gain here would be
        // silently ignored for this control path. The only real lever left for simulated response
        // speed is the placeholder mass/length below - it needs to be light enough that the real,
        // unmodified pivotP/current-limit/profile can actually move it at a reasonable rate.
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withGearing(new MechanismGearing(gearRatio))
        .withTrapezoidalProfile(RotationsPerSecond.of(pivotVelocity), RotationsPerSecondPerSecond.of(pivotAcceleration))
        .withStatorCurrentLimit(Amps.of(pivotCurrentLimit))
        .withIdleMode(MotorMode.BRAKE)
        // Placeholder arm mass for simulation physics only - not a measured value. Lightened twice
        // now (from 14in/2kg to 6in/0.5kg to this) since heavier guesses made the simulated pivot
        // move too slowly for the real (unmodified) pivotP/profile/current-limit to track well.
        .withMomentOfInertia(Inches.of(6), Kilograms.of(0.5))
        .withStartingPosition(Rotations.of(0))
        .withTelemetry("IntakePivot", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    m_PivotMotor = new TalonFXWrapper(m_IntakePivot, DCMotor.getKrakenX60(1), pivotMotorConfig);

    ArmConfig pivotArmConfig = new ArmConfig()
        // Placeholder arm length for simulation physics only - not a measured value.
        .withLength(Inches.of(6))
        // NOT forwardLimit/reverseLimit - those soft limits are disabled on the real robot today
        // (commented out in the original config) and don't actually cover the real travel range;
        // intakeOutPosition (-0.333) is well past reverseLimit (-0.163). These hard limits instead
        // span the actual named positions used below, with a little margin, so the simulated arm
        // can physically reach every position the real robot commands it to.
        .withHardLimits(Rotations.of(-0.34), Rotations.of(0.01));
    m_PivotArm = new Arm(pivotArmConfig, m_PivotMotor);

    SmartMotorControllerConfig rollerMotorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(rollerP, rollerI, rollerD)
        .withFeedforward(new SimpleMotorFeedforward(0, rollerV, 0))
        // No SensorToMechanismRatio was ever set for the rollers in the original config, which
        // means the real robot already runs them at CTRE's default 1:1 - not a placeholder here.
        .withGearing(MechanismGearing.kOne)
        .withStatorCurrentLimit(Amps.of(rollerCurrentLimit))
        .withSupplyCurrentLimit(Amps.of(rollerSupplyLimit))
        .withIdleMode(MotorMode.COAST)
        // Placeholder roller mass for simulation physics only - not a measured value.
        .withMomentOfInertia(Inches.of(1.5), Kilograms.of(0.3))
        .withTelemetry("IntakeRoller", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    m_RollerMotor = new TalonFXWrapper(m_RightIntakeRoller, DCMotor.getKrakenX60(1), rollerMotorConfig);

    // Placeholder roller diameter for simulation physics only - not a measured value.
    FlyWheelConfig rollerFlyWheelConfig = new FlyWheelConfig().withDiameter(Inches.of(2));
    m_RollerFlyWheel = new FlyWheel(rollerFlyWheelConfig, m_RollerMotor);

    // Set Intake RELATIVE
    m_IntakePivot.setPosition(0);
  }

  public void moveIntake(){
    m_PivotMotor.setPosition(Rotations.of(target));
  }

   public void setIntakePivotPosition(double setPosition)
   {
    m_IntakePivot.setPosition(setPosition);
    m_PivotMotor.setKp(pivotP);
   }

  public void homeIntakePivot()
  {
    m_PivotMotor.setKp(.25);
    m_PivotMotor.setVelocity(RotationsPerSecond.of(-3));
  }

  public void stopIntakePivot()
  {
    m_PivotMotor.setVelocity(RotationsPerSecond.of(0));
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

  public double getIntakeBlockPosition()
  {
    return intakeBlockPosition;
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
    m_PivotMotor.setMotionProfileMaxAcceleration(RotationsPerSecondPerSecond.of(pivotAccelerationSlow));
    // intakeOut = true;
  }

  public void setNormalMode()
  {
    m_PivotMotor.setMotionProfileMaxAcceleration(RotationsPerSecondPerSecond.of(pivotAcceleration));
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
    m_RollerMotor.setVelocity(RotationsPerSecond.of(speed));
    // NOTE: preserved as-is - this call was already missing .withLeaderID(rightIntakePowerID) in
    // the original code (every other roller method below sets it), so the left roller doesn't
    // actually follow here. Not fixed since setIntakeSpeed isn't used by any active binding today.
    m_LeftIntakeRoller.setControl(m_RollerFollower);
  }

  public void setIntakeVoltage(){
    m_RollerMotor.setVoltage(Volts.of(intakeVoltage));
    m_LeftIntakeRoller.setControl(m_RollerFollower.withLeaderID(rightIntakePowerID));
    intaking = true;
  }
  public void setOutakeVoltage() {
    m_RollerMotor.setVoltage(Volts.of(-intakeVoltage));
    m_LeftIntakeRoller.setControl(m_RollerFollower.withLeaderID(rightIntakePowerID));
    intaking = false;
  }
  public void setIdleVoltage(){
    m_RollerMotor.setVoltage(Volts.of(intakeVoltageIdle));
    m_LeftIntakeRoller.setControl(m_RollerFollower.withLeaderID(rightIntakePowerID));
    intaking = false;
  }
  public void stopIntakeVoltage() {
    m_RollerMotor.setVoltage(Volts.of(0.0));
    m_LeftIntakeRoller.setControl(m_RollerFollower.withLeaderID(rightIntakePowerID));
    intaking = false;
  }

  /**
   * Whether the intake rollers are currently commanded to run in the intaking direction. Used in
   * simulation to start/stop the maple-sim Fuel intake to match real intake state.
   */
  public boolean isIntaking()
  {
    return intaking;
  }

  public void setCurrentLimit(double currentLimit)
  {
    m_RollerMotor.setStatorCurrentLimit(Amps.of(currentLimit));
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
    m_PivotArm.updateTelemetry();
    m_RollerFlyWheel.updateTelemetry();

    // The CAD assembly was exported while posed in its deployed state (intakeOutPosition), not
    // real-world zero/stowed - so the mesh's own baked-in zero rotation corresponds to
    // intakeOutPosition, not 0. Publish the angle relative to that reference instead of raw
    // position, so config.json's zeroedRotations (aligning the as-exported deployed pose to the
    // robot frame) stays correct and this offset alone accounts for the CAD/real-zero mismatch.
    double angleRadians = -Rotations.of(getIntakePosition() - intakeOutPosition).in(Radians);
    intakeComponentPublisher.set(new Pose3d[] {
        new Pose3d(0, 0, 0, new Rotation3d(0, angleRadians, 0))
    });

    // Intake Logging
    SmartDashboard.putNumber("Intake Pivot Motor Position", getIntakePosition());
    SmartDashboard.putNumber("Intake Pivot Target", target);
     SmartDashboard.putNumber("Intake Pivot Motor Velocity", m_IntakePivot.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Intake Roller Speed", m_RightIntakeRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Intake Roller Speed", m_LeftIntakeRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Intake Roller Current", m_RightIntakeRoller.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left Intake Roller Current", m_LeftIntakeRoller.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Right Intake Roller Supply Current", m_RightIntakeRoller.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left Intake Roller Supply Current", m_LeftIntakeRoller.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Right Intake Roller Voltage", m_RightIntakeRoller.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Left Intake Roller Voltage", m_LeftIntakeRoller.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Current", m_IntakePivot.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    m_PivotArm.simIterate();
    m_RollerFlyWheel.simIterate();
  }
}
