// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimShooter;
import frc.robot.commands.HomeIntake;
import frc.robot.commands.HomeShooter;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.Outake;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.RunHotDog;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.commands.StowShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.HotDog;
// import frc.robot.subsystems.Lights;
import frc.robot.subsystems.swervedrive.Eyes;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.security.spec.NamedParameterSpec;

import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driver = new CommandXboxController(0);
  final CommandPS5Controller operator = new CommandPS5Controller(1);

  // driver is physically a PS5 DualSense, but declared as CommandXboxController - button indices
  // happen to line up, but its fixed Xbox axis map does not: leftTrigger()/rightTrigger() read raw
  // axes 2/3 (Xbox convention), but this controller's triggers are actually raw axes 3/4 (same
  // root cause as the drive rotation axis fix in SwerveSubsystem's default command). These replace
  // driver.leftTrigger()/rightTrigger() with the correct raw axes.
  final Trigger driverLeftTrigger = new Trigger(() -> driver.getRawAxis(3) > 0.5);
  final Trigger driverRightTrigger = new Trigger(() -> driver.getRawAxis(4) > 0.5);
  //final         CommandPS5Controller operator = new CommandPS5Controller(1);
  // The robot's subsystems and commands are defined here...
  private static final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/falcon"));
  
  public final Eyes s_Eyes = new Eyes(drivebase);
  public final Intake s_Intake = new Intake();
  public final ShooterPivot s_ShooterPivot = new ShooterPivot(s_Eyes);
  public final HotDog s_HotDog = new HotDog();
  public final Shooter s_Shooter = new Shooter(s_Eyes);
  // public final Lights s_Lights = new Lights();
  // public final Feeder s_Feeder = new Feeder();
  // public final Climber s_Climber = new Climber();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driver.getLeftY() * -1,
                                                                () -> driver.getLeftX() * -1)
                                                            // driver.getRightX() reads raw axis 4, the Xbox
                                                            // convention - but the driver station controller is a
                                                            // PS5 DualSense, whose right stick X sits at raw axis 2
                                                            // under Windows. Reading axis 4 there was reading
                                                            // whatever's actually at that slot (a trigger), causing
                                                            // spurious nonzero rotation with the stick untouched.
                                                            .withControllerRotationAxis(() -> -driver.getRawAxis(2))
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
   

      
       drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
      //  s_Shooter.setDefaultCommand(new InstantCommand(() -> s_Shooter.setShooterSpeed(-15)));



    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Auto Commands

    NamedCommands.registerCommand("Run HotDog", 
    new InstantCommand(() -> s_HotDog.setHotDogSpeed(s_HotDog.getHotDogSpeed())).raceWith(new WaitCommand(5)).alongWith(new InstantCommand(() -> s_HotDog.setIndexerSpeed(s_HotDog.getIndexerSpeed())).raceWith(new WaitCommand(5))));
    NamedCommands.registerCommand("Stop HotDog", new InstantCommand(() -> s_HotDog.setHotDogSpeed(0)).alongWith(new InstantCommand(() -> s_HotDog.setIndexerSpeed(0))));
    NamedCommands.registerCommand("Prepare Shooter", new PrepareShooter(s_Shooter));//-40.52
    NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> s_Shooter.setShooterVoltage(-1)));
    NamedCommands.registerCommand("Fart Shooter", new InstantCommand(() -> s_Shooter.setShooterVoltage(-2)));
    NamedCommands.registerCommand("Shooter Pivot Fart", new SetShooterAngle(s_ShooterPivot, -0.065));
    NamedCommands.registerCommand("Intake",
      new InstantCommand(() -> s_Intake.setIntakeVoltage())
    );
    NamedCommands.registerCommand("Stop Intake",
      new InstantCommand(() -> s_Intake.stopIntakeVoltage())
    );
    NamedCommands.registerCommand("Move Intake", new SequentialCommandGroup(
        new InstantCommand(() -> s_Intake.changeTarget()),
        new MoveIntake(s_Intake),
        new InstantCommand(() -> s_Intake.changeState())

      )); 
    

    NamedCommands.registerCommand("Aim Shooter", new AimShooter(s_ShooterPivot));//-0.141
    NamedCommands.registerCommand("Stow Shooter", new InstantCommand(() -> s_ShooterPivot.setShooterAngle(s_ShooterPivot.getShooterPivotSafePose())));
    NamedCommands.registerCommand("Lock Wheels", new InstantCommand(() -> drivebase.lock()).repeatedly());
    // NamedCommands.registerCommand("Run Feeder", new InstantCommand(() -> s_Feeder.setFeederSpeed(s_Feeder.getLeftFeederSpeed(),s_Feeder.getRightFeederSpeed())));
    // NamedCommands.registerCommand("Stop Feeder", new InstantCommand(() -> s_Feeder.setFeederSpeed(0,0)));
   NamedCommands.registerCommand("Pump Intake", new ParallelCommandGroup(
    //new InstantCommand(() -> s_Intake.setIntakeSpeed(40)),
   new SequentialCommandGroup(
        new InstantCommand(() -> s_Intake.setState(true)),
        new InstantCommand(() -> s_Intake.setIntakeTarget(s_Intake.getIntakePumpPosition())),
        new MoveIntake(s_Intake),
        new InstantCommand(() -> s_Intake.setState(false)),
        new InstantCommand(() -> s_Intake.changeTarget()),
        new MoveIntake(s_Intake),
        new InstantCommand(() -> s_Intake.changeState())



      ))); 

    NamedCommands.registerCommand("Auto Aim",
      drivebase.driveCommand(
          () -> 0,
          () -> 0,
          () -> (s_Eyes.getTargetRotation()) * (.12)).repeatedly().withTimeout(7));
    
    // Shuffleboard Auto Chooser
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    Command citrusRightTrench = new PathPlannerAuto("Citrus Left Trench", true);
    Command delayedRightBump = new PathPlannerAuto("Delayed Left Bump", true);
    Command poofsAutoRight = new PathPlannerAuto("Poofs Auto", true);
    autoChooser.addOption("Poofs Auto Right", poofsAutoRight);
    autoChooser.addOption("Citrus Right Trench Mirror", citrusRightTrench);
    autoChooser.addOption("Delayed Right Bump", delayedRightBump);
    

    // FollowPathCommand.warmupCommand();

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
     Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
    //     driveDirectAngle);
    // Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
    //     driveDirectAngleKeyboard);


    // Teleop Commands

    // Aiming Command
        driverLeftTrigger.whileTrue(
          new ParallelCommandGroup(
            driveFieldOrientedDirectAngle = drivebase.driveCommand(
          () -> driver.getLeftY(),
          () -> driver.getLeftX(),
          () -> drivebase.getRotationToFaceHub()),
            new AimShooter(s_ShooterPivot),
            new PrepareShooter(s_Shooter)
          ))
      .onFalse(
        new ParallelCommandGroup(
            new StowShooter(s_ShooterPivot),
            new InstantCommand(() -> s_Shooter.setShooterVoltage(-1)),
            new InstantCommand(() -> s_HotDog.setIndexerSpeed(0))

            //  new InstantCommand(() -> driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0))
        )
      );



      // driver.triangle().whileTrue(
      //   new ParallelCommandGroup(
      //      driveFieldOrientedDirectAngle = drivebase.driveCommand(
      //     () -> driver.getLeftY(),
      //     () -> driver.getLeftX(),
      //     () -> (s_Eyes.getDefenseRotation()) * (.12))       
      //     ))
      // .onFalse(
      //   new ParallelCommandGroup(
      //     driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity)
      //   )
      // );

      

  


    
    //zero gyro
    // This PS5 DualSense echoes the right trigger's digital click onto button 8 (confirmed via
    // Diag/ButtonsPressedDuringShoot), which is exactly the index start() reads - so pressing shoot
    // was silently also zeroing the gyro/resetting heading to a fixed 180 (the "shoot causes an
    // instant 180 snap" bug). A same-tick AND guard against the trigger didn't fully fix it (still
    // reproduced) - the digital echo appears to land on an earlier tick than the analog axis crosses
    // its deadband, slipping through before the guard engages. Moved off start()/button 8 entirely
    // onto the D-pad (a POV hat switch - a different HID report type from buttons/axes, so it can't
    // alias with a trigger echo at all) instead of chasing the exact timing.
    driver.povUp().onTrue(new InstantCommand(() -> drivebase.zeroGyroWithAlliance()));

    //spawn a test Fuel game piece in simulation, for manually testing intake capture
    driver.y().onTrue(new InstantCommand(() -> drivebase.spawnTestFuelGamePiece()));

    //run intake
    driver.leftBumper().whileTrue(new RunIntake(s_Intake));
    // InstantCommand(() -> s_Intake.setIntakeSpeed(s_Intake.getIntakeSpeed())))
    // .onFalse(new InstantCommand(() -> s_Intake.setIntakeSpeed(0)));

    //intake out
    driver.a().onTrue(
      new SequentialCommandGroup(
        // new InstantCommand(() -> s_Intake.setSlowMode()),
        new InstantCommand(() -> s_Intake.changeTarget()),
        new MoveIntake(s_Intake),
        new InstantCommand(() -> s_Intake.changeState())

      ).withTimeout(2.0)
    );

    //home shooter
    // driver.options().onTrue(
    //   new HomeShooter(s_ShooterPivot)
    // );

    //manual trench shot
    driver.b().whileTrue(

      new ParallelCommandGroup(
        new InstantCommand(() -> s_Shooter.setShooterSpeed(-47.75)),
        new SetShooterAngle(s_ShooterPivot, -0.034),//-0.034
        new InstantCommand(() -> s_HotDog.setIndexerSpeed(s_HotDog.getIndexerSpeed()))
      )
    ).onFalse(
      new ParallelCommandGroup(
        new InstantCommand(() -> s_Shooter.setShooterVoltage(-1)),
        new StowShooter(s_ShooterPivot),
        new InstantCommand(() -> s_HotDog.setIndexerSpeed(0))
        )
    );

//shoot
     driverRightTrigger.whileTrue(
      new ParallelCommandGroup(
        
        //new InstantCommand(() -> s_Intake.setCurrentLimit(s_Intake.getIdleRollerCurrentLimit())),
        //new InstantCommand(() -> s_Intake.setIntakeSpeed(20)),
        new RunHotDog(s_HotDog)
        // new SequentialCommandGroup(
        //   new WaitCommand(1.5),
        //   new InstantCommand(() -> s_Intake.setSlowMode()),
        //   new InstantCommand(() -> s_Intake.setIntakeTarget(s_Intake.getIntakeMiddlePosition())),
        //   new MoveIntake(s_Intake)
        ))
        .onFalse(
          new SequentialCommandGroup(
          new ParallelCommandGroup(
          // new InstantCommand(() -> s_Intake.setCurrentLimit(s_Intake.getIdleRollerCurrentLimit())),
          // new InstantCommand(() -> s_Intake.setIntakeSpeed(20))
        // new InstantCommand(() -> s_Intake.setTarget(s_Intake.getIntakePosition())),
        // new MoveIntake(s_Intake),

        // new InstantCommand(() -> s_Intake.setNormalMode())
        )
        //  new InstantCommand(() -> s_Intake.changeTarget()),
        // new MoveIntake(s_Intake),
        // new InstantCommand(() -> s_Intake.changeState())
        )
        );


//outake
    // driver.R1().whileTrue(
    //   new Outake(s_Intake, s_HotDog));


//pump intake
    driver.povUp().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> s_Intake.setIntakeSpeed(40)),
        new SequentialCommandGroup(
          // new InstantCommand(() -> s_Intake.setNormalMode()),
          new InstantCommand(() -> s_Intake.setState(true)),
          new InstantCommand(() -> s_Intake.setIntakeTarget(s_Intake.getIntakePumpPosition())),
          new MoveIntake(s_Intake)
        )
      )
        
    ).onFalse(
      new ParallelCommandGroup(
        new InstantCommand(() -> s_Intake.setIntakeSpeed(0)),
        new SequentialCommandGroup(
            new InstantCommand(() -> s_Intake.setState(false)),
            new InstantCommand(() -> s_Intake.changeTarget()),
            new MoveIntake(s_Intake),
            new InstantCommand(() -> s_Intake.changeState())
        )
      )
    );


    //Block
    driver.rightBumper().onTrue(
      new ParallelCommandGroup(
        //new InstantCommand(() -> s_Intake.setIntakeSpeed(40)),
        new SequentialCommandGroup(
          // new InstantCommand(() -> s_Intake.setNormalMode()),
          new InstantCommand(() -> s_Intake.setState(true)),
          new InstantCommand(() -> s_Intake.setIntakeTarget(s_Intake.getIntakeBlockPosition())),
          new MoveIntake(s_Intake)
        )
      )
        
    ).onFalse(
      new ParallelCommandGroup(
        //new InstantCommand(() -> s_Intake.setIntakeSpeed(0)),
        new SequentialCommandGroup(
            new InstantCommand(() -> s_Intake.setState(false)),
            new InstantCommand(() -> s_Intake.changeTarget()),
            new MoveIntake(s_Intake),
            new InstantCommand(() -> s_Intake.changeState())
        )
      )
    );

    
    // driver.triangle().onTrue(
    //   new InstantCommand(() -> s_Climber.setServo(s_Climber.getServoIn()))
    // );

    //pass
        driver.x().whileTrue(
          new ParallelCommandGroup(
          new InstantCommand(() -> s_Shooter.setShooterSpeed(-56)),
          new SetShooterAngle(s_ShooterPivot, -0.065),//-0.065
          new InstantCommand(() -> s_HotDog.setIndexerSpeed(s_HotDog.getIndexerSpeed()))     
          ))
      .onFalse(
        new ParallelCommandGroup(
            new StowShooter(s_ShooterPivot),
            new InstantCommand(() -> s_Shooter.setShooterVoltage(-1)),
            new InstantCommand(() -> s_HotDog.setIndexerSpeed(0))  
        )
      );
      

    // home Intake
    driver.povRight().onTrue(
      new HomeIntake(s_Intake)
    );


    //operator conctrols

    //reset Limelight
    operator.touchpad().onTrue(
      new InstantCommand(() -> Eyes.ResetLimelight())
    );

    //increase shooter speed
    operator.povUp().onTrue(
      new InstantCommand(() -> s_Shooter.increaseShooterSpeed())
    );

    //decrease shooter speed
    operator.povDown().onTrue(
      new InstantCommand(() -> s_Shooter.decreaseShooterSpeed())
    );

    //reset shooter speed
    operator.cross().onTrue(
      new InstantCommand(() -> s_Shooter.resetShooterSpeed())
    );

    if (SwerveDriveTelemetry.isSimulation)
    {
      configureMapleSimBindings();
    }
  }

  /**
   * Wires the maple-sim REBUILT Fuel intake/shooter simulation to the real Intake/HotDog subsystem
   * state, so simulation behavior tracks whatever the driver/operator actually command. Only called
   * in simulation.
   */
  private void configureMapleSimBindings()
  {
    new Trigger(s_Intake::isIntaking)
        .onTrue(new InstantCommand(() -> drivebase.setFuelIntakeRunning(true)))
        .onFalse(new InstantCommand(() -> drivebase.setFuelIntakeRunning(false)));

    // Indexer starting to run is treated as the moment a held Fuel piece gets fed into the spinning
    // shooter flywheel and launched. Launch angle/speed are approximated from the shooter pivot
    // setpoint and flywheel target speed - see SwerveSubsystem.fireFuelFromShooter for placeholder
    // physical constants that need tuning once real shooter geometry/muzzle velocity is known.
    new Trigger(s_HotDog::isIndexerRunning)
        .onTrue(new InstantCommand(
            () -> drivebase.fireFuelFromShooter(s_ShooterPivot.getShooterAngle(), s_Shooter.getTargetSpeed())));
  }

  

   public void rumbleControllers() {
        if (s_Shooter.isReadyToShoot())
        {
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
            // driver.getHID().setOutput(1, true);
        } 
        else 
        {
            driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            // driver.getHID().setOutput(0, false);
        } 
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

  public static SwerveSubsystem getSwerveDrive()
  {
    return drivebase;
  }
}
