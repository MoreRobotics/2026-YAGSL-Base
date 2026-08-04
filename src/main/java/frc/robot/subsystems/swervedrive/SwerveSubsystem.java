// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.dyn4j.dynamics.Body;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  public final SwerveDrive swerveDrive;
  /**
   * Enable vision odometry updates while driving.
   */
  private final boolean     visionDriveTest = false;
  
  // public StructPublisher<Pose2d> estimatedRobotPosePublisher;
   public SwerveDrivePoseEstimator m_PoseEstimator;
   
  /**
   * PhotonVision class to keep an accurate odometry.
   */
  private       Eyes      s_Eyes;

  private PathPlannerTrajectory pathPlannerTrajectory;

  public boolean redAlliance;

  private RebuiltFuelOnField trackedTestFuelPiece;
  private Translation2d trackedTestFuelSpawnLocation;
  private double trackedTestFuelSpawnTimestamp;

  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  /**
   * Standalone maple-sim drivetrain used only as a positional anchor for the REBUILT Fuel intake
   * simulation below. YAGSL bundles its OWN shaded/relocated copy of maple-sim internally
   * (a separate class hierarchy and physics world from this one - confirmed by a compile error when
   * an attempt was made to reuse {@link SwerveDrive#getMapleSimDrive()} directly here instead), so a
   * separate instance genuinely is required to give {@link IntakeSimulation} something to attach to
   * in THIS arena. Its pose is overwritten every simulation tick to match YAGSL's real simulated
   * robot pose. Null outside of simulation.
   */
  private SwerveDriveSimulation mapleSimIntakeAnchor;

  /**
   * Simulated intake for REBUILT Fuel game pieces. Null outside of simulation.
   */
  private IntakeSimulation fuelIntakeSimulation;

  // Placeholder shooter geometry/performance constants for simulated Fuel launches - none of these
  // are measured from the real robot yet. Tune once the shooter's physical mount point, exit
  // height, and true muzzle velocity/angle range are characterized.
  // The shooter is mounted on the back of the robot, opposite the front-mounted intake - negative X
  // (behind robot center in robot-forward coordinates) accounts for that.
  private static final Translation2d SHOOTER_POSITION_ON_ROBOT = new Translation2d(Inches.of(-13), Inches.of(0));
  private static final Distance      SHOOTER_EXIT_HEIGHT        = Inches.of(24);
  // ShooterPivot.getShooterAngle() operating range (rotations), see ShooterPivot's clamp bounds.
  private static final double        PIVOT_MIN_ROTATIONS = -0.071;
  private static final double        PIVOT_MAX_ROTATIONS = -0.004;
  private static final Angle         LAUNCH_ANGLE_AT_PIVOT_MIN = Degrees.of(55);
  private static final Angle         LAUNCH_ANGLE_AT_PIVOT_MAX = Degrees.of(20);
  // Real REBUILT hub scoring height (matches frc1678's public kHubAltitude constant, cross-checked
  // against real field geometry) - used for ballistic-solving launch speed below, not a guess.
  private static final Distance      HUB_TARGET_HEIGHT = Inches.of(72);
  // Fuel-piece-diameter-ish flywheel effective radius used to convert commanded flywheel
  // rotations/sec into an exit speed; not a measured value.
  private static final double        FLYWHEEL_EFFECTIVE_RADIUS_METERS = Units.inchesToMeters(2);

  /**
   * Publishes on-field and in-flight Fuel piece poses for visualization (e.g. AdvantageScope's 3D
   * Field view). Null outside of simulation.
   */
  private StructArrayPublisher<Pose3d> fuelPosesPublisher;

  /**
   * Publishes the robot's own pose for visualization (e.g. AdvantageScope's 3D Field view). Nothing
   * else in this codebase publishes a robot pose struct.
   */
  private final StructPublisher<Pose3d> robotPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Telemetry/RobotPose3d", Pose3d.struct).publish();

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    redAlliance = false;
    // boolean blueAlliance = false;
    
    //  Pose2d startingPose = 
    //  !redAlliance ? new Pose2d(new Translation2d(Meter.of(1),
    //                                                                   Meter.of(4)),
    //                                                 Rotation2d.fromDegrees(0))
    //                                    : new Pose2d(new Translation2d(Meter.of(16),
    //                                                                   Meter.of(4)),
    //                                                 Rotation2d.fromDegrees(180));

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, new Pose2d());
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    //RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    if (SwerveDriveTelemetry.isSimulation)
    {
      // Simulation-only: auto-zero the gyro/heading whenever entering teleop, so testing doesn't
      // depend on remembering to press the manual zero-gyro binding first. Gated to simulation so
      // this doesn't change real-robot behavior (a real match shouldn't silently reset pose on every
      // enable).
      RobotModeTriggers.teleop().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    }

    m_PoseEstimator =
        new SwerveDrivePoseEstimator(
            getKinematics(),
            getHeading(),
            getSwerveDrive().getModulePositions(),
            getPose(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(.7, .7, 99)
            );
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    setupPathPlanner();

    if (SwerveDriveTelemetry.isSimulation)
    {
      setupMapleSimIntake();
    }
  }

  /**
   * Sets up the standalone maple-sim drivetrain anchor and REBUILT Fuel intake simulation.
   * See the {@link #mapleSimIntakeAnchor} field doc for why this is separate from YAGSL's own
   * bundled maple-sim integration.
   */
  private void setupMapleSimIntake()
  {
    // The generic SimulatedArena has no REBUILT-specific scoring: RebuiltHub (which detects a Fuel
    // piece flying into the hub and calls Arena2026Rebuilt.addToScore(...)) only exists on this
    // season-specific arena subclass. Must override the singleton before anything below registers
    // to it, or scoring silently never happens despite otherwise-correct shot physics.
    SimulatedArena.overrideInstance(new Arena2026Rebuilt());
    SimulatedArena.getInstance().enableBreakdownPublishing();

    // YAGSL bundles its OWN shaded/relocated copy of maple-sim (swervelib.simulation.ironmaple...)
    // for its internal drivetrain physics - a completely separate class hierarchy and physics world
    // from the org.ironmaple... classes used everywhere else here (arena, game pieces). The two
    // never interact, so reusing swerveDrive.getMapleSimDrive() directly isn't possible (confirmed by
    // a compile error: incompatible types). A separate standalone body, glued to the real pose every
    // tick, is genuinely necessary here - it's not the cause of the uncommanded spin.
    DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(67.13))
        .withBumperSize(Inches.of(34.75), Inches.of(34.75))
        .withTrackLengthTrackWidth(Inches.of(22.73), Inches.of(22.73))
        // COTS.ofMark4i bakes in stock MK4i steer ratio (21.4285714286:1) and 4in wheel diameter,
        // matching the real robot. Drive/steer friction voltage (0.1V/0.2V) and steer MOI
        // (0.03 kg*m^2) are that preset's built-in placeholder defaults - tuning knobs for later,
        // not correctness issues. "2" selects the MK4i L2 gear ratio (6.75:1) to match the real robot.
        .withSwerveModule(COTS.ofMark4i(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), 1.19, 2))
        .withGyro(COTS.ofPigeon2());

    mapleSimIntakeAnchor = new SwerveDriveSimulation(mapleSimConfig, getPose());
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimIntakeAnchor);

    // Widened from the original 26in/12in - capture was reported inconsistent (works sometimes,
    // not others, with no clear pattern), consistent with a capture zone that's narrow relative to
    // normal driving precision. Not a measured/tuned value, just more forgiving.
    fuelIntakeSimulation = IntakeSimulation.OverTheBumperIntake(
        RebuiltFuelOnField.REBUILT_FUEL_INFO.type(),
        mapleSimIntakeAnchor,
        Inches.of(32),
        Inches.of(20),
        IntakeSimulation.IntakeSide.FRONT,
        50);
    // Tried marking this a sensor (dyn4j BodyFixture.setSensor) to stop it physically ramming Fuel
    // pieces instead of smoothly capturing them - reverted. maple-sim's own IntakeSimulation source
    // deliberately does NOT do this, and after adding it, capture became drastically less reliable
    // (many minutes or never, instead of "works sometimes, no clear pattern"). Whatever the exact
    // mechanism, the physical-push behavior is the lesser problem - capture actually working is the
    // priority.
    fuelIntakeSimulation.register();
    // Starts stopped; the real Intake subsystem's roller state drives this via setFuelIntakeRunning().

    fuelPosesPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MapleSim/FuelPoses3d", Pose3d.struct).publish();
  }

  /**
   * Starts or stops the maple-sim Fuel intake to match the real {@link frc.robot.subsystems.Intake}
   * subsystem's roller state. No-op outside of simulation.
   */
  public void setFuelIntakeRunning(boolean running)
  {
    if (fuelIntakeSimulation == null)
    {
      return;
    }
    if (running)
    {
      fuelIntakeSimulation.startIntake();
    } else
    {
      fuelIntakeSimulation.stopIntake();
    }
  }

  /**
   * If the simulated intake is holding a Fuel piece, removes it and launches it as a projectile
   * approximating a real shot, using the shooter pivot setpoint and flywheel target speed. No-op
   * outside of simulation or if no piece is held. See the placeholder constants above this class's
   * maple-sim fields for the (untuned) shooter geometry/performance this approximation relies on.
   *
   * @param pivotPositionRotations Current {@link frc.robot.subsystems.ShooterPivot#getShooterAngle()} setpoint.
   * @param flywheelTargetSpeedRps Current {@link frc.robot.subsystems.Shooter#getTargetSpeed()} setpoint.
   */
  /**
   * Rotational velocity (radians/sec) to command so the robot turns to face the shooter (mounted on
   * the back) at the real hub, using the same continuous-input wrapped {@link #headingController}
   * as trajectory following - replaces {@link frc.robot.subsystems.swervedrive.Eyes#getTargetRotation()}'s
   * manual per-alliance degree math, which had no wraparound and could snap to the wrong heading
   * (e.g. facing the driver station) whenever the raw bearing crossed +/-180.
   */
  public double getRotationToFaceHub()
  {
    Pose2d robotPose = getPose();
    Translation2d hubPosition = redAlliance
        ? new Translation2d(Constants.Positions.hubRedX, Constants.Positions.hubRedY)
        : new Translation2d(Constants.Positions.hubBlueX, Constants.Positions.hubBlueY);
    Rotation2d bearingToHub = new Rotation2d(
        hubPosition.getX() - robotPose.getX(), hubPosition.getY() - robotPose.getY());
    Rotation2d targetHeading = bearingToHub.plus(Rotation2d.fromDegrees(180));
    return headingController.calculate(robotPose.getRotation().getRadians(), targetHeading.getRadians());
  }

  public void fireFuelFromShooter(double pivotPositionRotations, double flywheelTargetSpeedRps)
  {
    if (fuelIntakeSimulation == null || fuelIntakeSimulation.getGamePiecesAmount() <= 0)
    {
      return;
    }
    fuelIntakeSimulation.obtainGamePieceFromIntake();

    Angle launchAngle = Degrees.of(MathUtil.interpolate(
        LAUNCH_ANGLE_AT_PIVOT_MIN.in(Degrees),
        LAUNCH_ANGLE_AT_PIVOT_MAX.in(Degrees),
        MathUtil.inverseInterpolate(PIVOT_MIN_ROTATIONS, PIVOT_MAX_ROTATIONS, pivotPositionRotations)));

    // Ballistic-solve launch speed to actually reach the real hub position/height, instead of
    // guessing exit speed from flywheel RPS via an uncalibrated wheel-radius conversion. Hub
    // coords (Constants.Positions.hubBlueX/Y, hubRedX/Y) are real field geometry, cross-checked
    // against frc1678's public field-layout constants - not a guess. Distance uses robot center
    // (not the shooter's offset exit point) to match Eyes.getTargetDistance()'s own approximation.
    Pose2d robotPose = getPose();
    Translation2d hubPosition = redAlliance
        ? new Translation2d(Constants.Positions.hubRedX, Constants.Positions.hubRedY)
        : new Translation2d(Constants.Positions.hubBlueX, Constants.Positions.hubBlueY);
    double horizontalDistanceMeters = robotPose.getTranslation().getDistance(hubPosition);
    double heightDeltaMeters = HUB_TARGET_HEIGHT.in(Meter) - SHOOTER_EXIT_HEIGHT.in(Meter);
    double angleRadians = launchAngle.in(Radians);
    double denominator = 2 * Math.cos(angleRadians) * Math.cos(angleRadians)
        * (horizontalDistanceMeters * Math.tan(angleRadians) - heightDeltaMeters);
    LinearVelocity launchSpeed;
    if (denominator > 0.01)
    {
      double v0 = Math.sqrt(9.81 * horizontalDistanceMeters * horizontalDistanceMeters / denominator);
      launchSpeed = MetersPerSecond.of(v0);
    } else
    {
      // Pivot angle too shallow to reach this distance/height at all - fall back to the old
      // flywheel-RPS-based guess rather than producing a NaN/negative speed.
      launchSpeed = MetersPerSecond.of(
          Math.abs(flywheelTargetSpeedRps) * FLYWHEEL_EFFECTIVE_RADIUS_METERS * 2 * Math.PI);
    }

    // Shooter faces the robot's back (opposite the intake's front), hence the 180 degree flip -
    // matches SHOOTER_POSITION_ON_ROBOT's negative X offset above.
    Rotation2d shooterFacing = robotPose.getRotation().plus(Rotation2d.fromDegrees(180));
    RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
        robotPose.getTranslation(),
        SHOOTER_POSITION_ON_ROBOT,
        getRobotVelocity(),
        shooterFacing,
        SHOOTER_EXIT_HEIGHT,
        launchSpeed,
        launchAngle);
    projectile.launch();
    SimulatedArena.getInstance().addGamePieceProjectile(projectile);
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));

    // YAGSL's built-in heading correction (docs: "should only be used while controlling the robot
    // via angle") kicks in on ANY drive call - AIM's or the default's - whenever rotation input is
    // near zero while translating: it silently latches whichever heading the robot had at that
    // instant and drives back to it via its own internal PID, independent of whatever Command is
    // actually scheduled. Every drive command here is angular-velocity-based (never heading-axis),
    // so this must stay off or it fights our own rotation commands / holds a stale AIM heading.
    swerveDrive.setHeadingCorrection(false);

    isRedAlliance();

    m_PoseEstimator = 
        new SwerveDrivePoseEstimator(
            getKinematics(),
            getHeading(),
            getSwerveDrive().getModulePositions(),
            getPose(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(.7, .7, 99)
            );

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    
  
    
  }

  /**
   * Setup the photon vision class.
   */


  @Override
  public void periodic()
  {
    m_PoseEstimator.update(getHeading(), swerveDrive.getModulePositions());
    robotPosePublisher.set(new Pose3d(getPose()));
  }

  @Override
  public void simulationPeriodic()
  {
    m_PoseEstimator.update(getHeading(), swerveDrive.getModulePositions());
    swerveDrive.updateOdometry();
    // estimatedRobotPosePublisher.set(m_PoseEstimator.getEstimatedPosition());

    if (mapleSimIntakeAnchor != null)
    {
      // Keep the intake anchor glued to the robot's real tracked pose. Previously preferred
      // getSimulationDriveTrainPose() (YAGSL's own internal, separately-shaded maple-sim
      // drivetrain) over getPose(), on the assumption it was live ground truth - but nothing in
      // this project ever set up that internal integration, so it very likely returns a stale/
      // default pose wrapped in a present Optional, not empty, meaning the .orElse(getPose())
      // fallback never actually ran. That would explain total non-collision for the entire
      // session (the anchor silently pinned near a fixed bad location, never moving with the
      // real robot) and the newly-observed test-piece spawning off the field. getPose() is the
      // one confirmed all session to track the robot correctly (AdvantageScope renders it, every
      // earlier diagnostic read it successfully) - using it unconditionally here instead.
      mapleSimIntakeAnchor.setSimulationWorldPose(swerveDrive.getPose());
      // Was removed earlier this session under the theory that YAGSL's own updateOdometry() already
      // ticks this same arena, making this redundant ("double-ticking"). That theory is wrong: YAGSL
      // bundles its OWN separately-shaded copy of maple-sim (confirmed by a compile error when trying
      // to directly reuse its internal drivetrain body here) - a completely different physics world
      // from this one, which holds the Fuel pieces, hub, and intake anchor. Nothing else ticks this
      // arena, so removing this call meant it was never stepped at all - explaining total non-
      // collision (shooting kept working because projectile flight is pure analytic kinematics that
      // doesn't need the world to step).
      SimulatedArena.getInstance().simulationPeriodic();
      SmartDashboard.putNumber("MapleSim/Fuel Pieces Held", fuelIntakeSimulation.getGamePiecesAmount());
      var fuelOnField = SimulatedArena.getInstance().getGamePiecesByType(RebuiltFuelOnField.REBUILT_FUEL_INFO.type());
      SmartDashboard.putNumber("MapleSim/Fuel Pieces On Field", fuelOnField.size());
      // Every Fuel piece on the field defaults to dyn4j's normal at-rest sleeping behavior, which
      // puts a freshly spawned (near-zero velocity) piece to sleep almost immediately - it then
      // looks frozen until some collision wakes it. Covers every spawn path (test-spawn button,
      // any future match-start field population), not just the one call site that creates them.
      for (var piece : fuelOnField)
      {
        if (piece instanceof Body body)
        {
          body.setAtRestDetectionEnabled(false);
          // Disabling detection alone only stops it from going to sleep in the future - if it's
          // already asleep from before this ran, force it awake now too.
          body.setAtRest(false);
        }
      }
      SmartDashboard.putNumber("MapleSim/Score", SimulatedArena.getInstance().getScore(!redAlliance));

      List<Pose3d> fuelPoses = new ArrayList<>(Arrays.asList(SimulatedArena.getInstance()
          .getGamePiecesArrayByType(RebuiltFuelOnField.REBUILT_FUEL_INFO.type())));
      for (var projectile : SimulatedArena.getInstance().gamePieceLaunched())
      {
        if (RebuiltFuelOnField.REBUILT_FUEL_INFO.type().equals(projectile.getType()))
        {
          fuelPoses.add(projectile.getPose3d());
        }
      }
      fuelPosesPublisher.set(fuelPoses.toArray(new Pose3d[0]));

      // Precise, non-subjective measurement of how long a spawned test piece takes to first budge
      // from its spawn point (from any cause - collision, gravity settling, anything) instead of
      // relying on a "feels like minutes" impression.
      if (trackedTestFuelPiece != null)
      {
        Translation2d currentLocation = trackedTestFuelPiece.getPose3d().getTranslation().toTranslation2d();
        if (currentLocation.getDistance(trackedTestFuelSpawnLocation) > 0.02)
        {
          SmartDashboard.putNumber("Diag/TestFuelSecondsToFirstMove",
              Timer.getFPGATimestamp() - trackedTestFuelSpawnTimestamp);
          trackedTestFuelPiece = null;
        }
      }
    }
  }

  /**
   * Spawns a test REBUILT Fuel game piece about 1 meter in front of the robot, for manually testing
   * intake capture in simulation. No-op outside of simulation.
   */
  public void spawnTestFuelGamePiece()
  {
    if (!SwerveDriveTelemetry.isSimulation)
    {
      return;
    }
    // Same pose source the intake anchor is now synced to (see mapleSimIntakeAnchor's field doc) -
    // getPose(), not getSimulationDriveTrainPose(), which was very likely returning a stale/default
    // pose (explains the piece spawning off the field when this used it).
    Pose2d robotPose = getPose();
    Translation2d spawnLocation = robotPose.getTranslation()
        .plus(new Translation2d(1.0, 0.0).rotateBy(robotPose.getRotation()));
    RebuiltFuelOnField fuel = new RebuiltFuelOnField(spawnLocation);
    // GamePieceOnFieldSimulation extends dyn4j's Body, which puts bodies to sleep (stops simulating
    // them) almost immediately once at rest - a freshly spawned piece has ~zero velocity, so it goes
    // dormant right away and only wakes when a force/impulse (e.g. a collision) hits it. Disabling
    // at-rest detection keeps it live from the moment it spawns instead of appearing frozen.
    fuel.setAtRestDetectionEnabled(false);
    SimulatedArena.getInstance().addGamePiece(fuel);
    trackedTestFuelPiece = fuel;
    trackedTestFuelSpawnLocation = spawnLocation;
    trackedTestFuelSpawnTimestamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Diag/TestFuelSecondsToFirstMove", -1.0);
  }

  public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        swerveDrive.driveFieldOriented(speeds);
    }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              redAlliance = alliance.get() == DriverStation.Alliance.Red;
              return redAlliance;
            }
            return redAlliance;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    //PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @return A {@link Command} which will run the alignment.
   */
   public Command aimAtTarget()
  {

    return run(() -> {
      
       
          drive(getTargetSpeeds(0,
                                0,
                                Rotation2d.fromDegrees(s_Eyes.getTargetRotation()))); // Not sure if this will work, more math may be required.
      
    });
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
  throws IOException, ParseException
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                            swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint
        = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                   swerveDrive.getStates(),
                                                   DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                    () -> {
                      double newTime = Timer.getFPGATimestamp();
                      SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                      robotRelativeChassisSpeed.get(),
                                                                                      newTime - previousTime.get());
                      swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                        newSetpoint.moduleStates(),
                                        newSetpoint.feedforwards().linearForces());
                      prevSetpoint.set(newSetpoint);
                      previousTime.set(newTime);

                    });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e)
    {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }


  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    DoubleSupplier newTranslationX;
    DoubleSupplier newTranslationY;
    isRedAlliance();
    if(!redAlliance)
    {
      newTranslationX = () -> -translationX.getAsDouble();
      newTranslationY = () -> -translationY.getAsDouble();
    }
    else
    {
      newTranslationX = () -> translationX.getAsDouble();
      newTranslationY = () -> translationY.getAsDouble();
    }


    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            MathUtil.applyDeadband(newTranslationX.getAsDouble(), Constants.OperatorConstants.DEADBAND) * swerveDrive.getMaximumChassisVelocity(),
                            MathUtil.applyDeadband(newTranslationY.getAsDouble(), Constants.OperatorConstants.DEADBAND) * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 1),
                        true,
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
    m_PoseEstimator.resetPosition(swerveDrive.getOdometryHeading(),swerveDrive.getModulePositions(), initialHolonomicPose);
  }


  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public Pose2d getEstimatedPosition()
  {
    return m_PoseEstimator.getEstimatedPosition();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public void isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent())
    {
      redAlliance = alliance.get() == DriverStation.Alliance.Red;
    }
    
  }



  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (redAlliance)
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();

    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}
