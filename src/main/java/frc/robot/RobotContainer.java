// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.swervedrive.Eyes;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
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
      

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

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

    // if (RobotBase.isSimulation())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    // } else
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // }

    // if (Robot.isSimulation())
    // {
    //   Pose2d target = new Pose2d(new Translation2d(1, 4),
    //                              Rotation2d.fromDegrees(90));
    //   //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
    //   driveDirectAngleKeyboard.driveToPose(() -> target,
    //                                        new ProfiledPIDController(5,
    //                                                                  0,
    //                                                                  0,
    //                                                                  new Constraints(5, 2)),
    //                                        new ProfiledPIDController(5,
    //                                                                  0,
    //                                                                  0,
    //                                                                  new Constraints(Units.degreesToRadians(360),
    //                                                                                  Units.degreesToRadians(180))
    //                                        ));
    //   driver.triangle().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    //   // driver.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    //   // driver.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
    //                                                 //  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));



    // }

    if (drivebase.isRedAlliance())
    {
      if(s_Eyes.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees() > 0)
      {
        driver.L2().whileTrue(
      driveFieldOrientedDirectAngle = drivebase.driveCommand(
          () -> -driver.getLeftY(),
          () -> -driver.getLeftX(),
          () -> (s_Eyes.getTargetRotation()-(180 - s_Eyes.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees())) * (-.1)))
      .onFalse(
        driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity)
      );
      }
      else {
        driver.L2().whileTrue(
      driveFieldOrientedDirectAngle = drivebase.driveCommand(
          () -> -driver.getLeftY(),
          () -> -driver.getLeftX(),
          () -> (s_Eyes.getTargetRotation()-(-180 - s_Eyes.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees())) * (-.1)))
      .onFalse(
        driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity)
      );
      }
    }
    else
    {
      driver.L2().whileTrue(
      driveFieldOrientedDirectAngle = drivebase.driveCommand(
          () -> -driver.getLeftY(),
          () -> -driver.getLeftX(),
          () -> (s_Eyes.getTargetRotation()-s_Eyes.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()) * (.1)))
      .onFalse(
        driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity)
      );
    }

    // driver.L2().whileTrue(
    //   driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //       () -> -driver.getLeftY(),
    //       () -> -driver.getLeftX(),
    //       () -> (s_Eyes.getTargetRotation()-s_Eyes.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()) * (.1)))
    //   .onFalse(
    //     driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity)
    //   );


    driver.options().onTrue(new InstantCommand(() -> drivebase.zeroGyroWithAlliance()));

   

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
