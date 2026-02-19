// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimShooter;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.Outake;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.RunHotDog;
import frc.robot.commands.RunIntake;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.HotDog;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.swervedrive.Eyes;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.units.Units.RPM;

import java.io.File;
import java.time.Instant;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandPS5Controller driver = new CommandPS5Controller(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/falcon"));
  
  public final Eyes s_Eyes = new Eyes(drivebase);
  public final HotDog s_HotDog = new HotDog();
  public final Intake s_Intake = new Intake();
  public final ShooterPivot s_ShooterPivot = new ShooterPivot(s_Eyes);
  public final Shooter s_Shooter = new Shooter(s_Eyes);
  public final Climber s_Climber = new Climber();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driver.getLeftY() * -1,
                                                                () -> driver.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> -driver.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
                                                                                             driver::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driver.getLeftY(),
                                                                        () -> -driver.getLeftX())
                                                                    .withControllerRotationAxis(() -> driver.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driver.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driver.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  public final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity);
   

      // drivebase.setDefaultCommand(
      //   new AbsoluteFieldDrive(
      //     drivebase, 
      //     () -> driver.getLeftX(),
      //     () -> driver.getLeftY(),
      //     () -> driver.getRightX()));
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

      //s_ShooterPivot.setDefaultCommand(new AimShooter(s_ShooterPivot,s_ShooterPivot.getShooterAngle()).repeatedly());

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("Auto Aim Blue", drivebase.aimAtTarget());
    NamedCommands.registerCommand("Run HotDog", new RunHotDog(s_HotDog));
    NamedCommands.registerCommand("PrepareShooter", new PrepareShooter(s_Shooter));
    NamedCommands.registerCommand("Intake", new RunIntake(s_Intake));
    NamedCommands.registerCommand("Move Intake", new MoveIntake(s_Intake)); 
    NamedCommands.registerCommand("Aim", new AimShooter(s_ShooterPivot));
    

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);


    if (drivebase.isRedAlliance())
    {
        driver.L2().whileTrue(
          new ParallelCommandGroup(
            driveFieldOrientedDirectAngle = drivebase.driveCommand(
          () -> driver.getLeftY(),
          () -> driver.getLeftX(),
          () -> (s_Eyes.getTargetRotation()) * (.12)),
          new AimShooter(s_ShooterPivot),
          new PrepareShooter(s_Shooter)
          ))
      .onFalse(
        new ParallelCommandGroup(
          driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity)
          // new AimShooter(s_ShooterPivot),
          //  new InstantCommand(() -> s_Shooter.setShooterSpeed(0))
        )
      );
      
      
    }
    else
    {
      driver.L2().whileTrue(
        new ParallelCommandGroup(
          driveFieldOrientedDirectAngle = drivebase.driveCommand(
          () -> -driver.getLeftY(),
          () -> -driver.getLeftX(),
          () -> (s_Eyes.getTargetRotation()-drivebase.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()) * (.12)),
          new AimShooter(s_ShooterPivot),
          new PrepareShooter(s_Shooter)
          ))
      .onFalse(
        new ParallelCommandGroup(
          driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity)
          // new AimShooter(s_ShooterPivot),
          //  new InstantCommand(() -> s_Shooter.setShooterSpeed(0))
          )
      );
    }

    // driver.L2().onTrue(
    //   new InstantCommand(() -> s_Shooter.setShooterSpeed(s_Shooter.getShooterSpeed()))
    // )
    // .onFalse(
    //   new InstantCommand(() -> s_Shooter.setShooterSpeed(0))
    // );

    // driver.L2().whileTrue(
    //   driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //       () -> -driver.getLeftY(),
    //       () -> -driver.getLeftX(),
    //       () -> (s_Eyes.getTargetRotation()-s_Eyes.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()) * (.1)))
    //   .onFalse(
    //     driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity)
    //   );


    
    //zero gyro
    driver.options().onTrue(new InstantCommand(() -> drivebase.zeroGyroWithAlliance()));

    //run intake
    driver.R1().whileTrue(new RunIntake(s_Intake));
    // InstantCommand(() -> s_Intake.setIntakeSpeed(s_Intake.getIntakeSpeed())))
    // .onFalse(new InstantCommand(() -> s_Intake.setIntakeSpeed(0)));

    //intake out
    driver.L1().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> s_Intake.changeTarget()),
        new MoveIntake(s_Intake),
        new InstantCommand(() -> s_Intake.changeState())

      )
    );

//shoot
     driver.R2().whileTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> s_Intake.setIntakeSpeed(s_Intake.getIntakeSpeed())),
        new RunHotDog(s_HotDog),
        new InstantCommand(() -> s_Intake.setSlowMode()),
        new InstantCommand(() -> s_Intake.changeTarget()),
        new MoveIntake(s_Intake)
        ))
      
        .onFalse(new ParallelCommandGroup(
          new InstantCommand(() -> s_Intake.setIntakeSpeed(0)),
        new InstantCommand(() -> s_Intake.setTarget(s_Intake.getIntakePosition())),
        new MoveIntake(s_Intake),
        new InstantCommand(() -> s_Intake.setNormalMode())));


//outake
    driver.cross().whileTrue(
      new Outake(s_Intake, s_HotDog));


 //Elevator up   
    driver.triangle().onTrue(
      new SequentialCommandGroup(
        new MoveClimber(s_Climber, s_Climber.getClimberHalfWayPose()),
        new ParallelCommandGroup(
          new InstantCommand(() -> s_Climber.setServo(s_Climber.getServoMid())),
          new MoveClimber(s_Climber, s_Climber.getClimberUpPose())
        ),
        new InstantCommand(() -> s_Climber.setServo(s_Climber.getServoOut()))
      )
    );


//Elevator Down
    driver.square().onTrue(
    new MoveClimber(s_Climber, s_Climber.getClimberDownPose())
    );

  }

   

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
