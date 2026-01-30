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

  // private double kP = 0;
  // private double kI = 0;
  // private double kD = 0;
  // private double maxVelocity = 0;//90
  // private double maxAcceleration = 0;//45
  // public double speed = 0;

  // private int iD = 0;
  // private double gearRatio = 0;

  // private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  // .withControlMode(ControlMode.CLOSED_LOOP)
  // // Feedback Constants (PID Constants)
  // .withClosedLoopController(kP, kI, kD, DegreesPerSecond.of(maxVelocity), DegreesPerSecondPerSecond.of(maxAcceleration))
  // .withSimClosedLoopController(kP, kI, kD, DegreesPerSecond.of(maxVelocity), DegreesPerSecondPerSecond.of(maxAcceleration))
  // // Feedforward Constants
  // .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  // .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  // // Telemetry name and verbosity level
  // .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // // Gearing from the motor rotor to final shaft.
  // // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // // You could also use .withGearing(12) which does the same thing.
  // .withGearing(new MechanismGearing(GearBox.fromReductionStages(gearRatio)))
  // // Motor properties to prevent over currenting.
  // .withMotorInverted(false)
  // .withIdleMode(MotorMode.COAST)
  // .withStatorCurrentLimit(Amps.of(40));

  // private TalonFXS m_Shooter = new TalonFXS(iD);

  // private SmartMotorController sparkSmartMotorController = new TalonFXSWrapper(m_Shooter, DCMotor.getKrakenX60(1), smcConfig);

  // private final FlyWheelConfig shooterConfig = new FlyWheelConfig()
  // // Diameter of the flywheel.
  // .withDiameter(Inches.of(4))
  // // Mass of the flywheel.
  // .withMass(Pounds.of(1))
  // // Maximum speed of the shooter.
  // .withUpperSoftLimit(RPM.of(1000))
  // // Telemetry name and verbosity for the arm.
  // .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  // // Shooter Mechanism
  // private FlyWheel shooter = new FlyWheel(shooterConfig);


  //  /**
  //  * Gets the current velocity of the shooter.
  //  *
  //  * @return Shooter velocity.
  //  */
  // public AngularVelocity getVelocity() {return shooter.getSpeed();}

  // /**
  //  * Set the shooter velocity.
  //  *
  //  * @param speed Speed to set.
  //  * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
  //  */
  // public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

  // /**
  //  * Set the dutycycle of the shooter.
  //  *
  //  * @param dutyCycle DutyCycle to set.
  //  * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
  //  */
  // public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

  /** Creates a new Shooter. */
  public Shooter() {
    
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
