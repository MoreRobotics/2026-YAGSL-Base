package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.ShooterPivot;

public class Aim extends Command {
    Lights s_Lights;
    SwerveSubsystem s_Swerve;
    ShooterPivot s_ShooterPivot;
    Eyes s_Eyes;

    double swerveTolerance = 2.0;
    double hoodTolerance = 1.0;

  /** Creates a new MoveIntake. */
    public Aim(Lights s_Lights, SwerveSubsystem s_Swerve, ShooterPivot s_ShooterPivot, Eyes s_Eyes) {
        this.s_Lights = s_Lights;
        this.s_Swerve = s_Swerve;
        this.s_ShooterPivot = s_ShooterPivot;
        this.s_Eyes = s_Eyes;

        addRequirements(s_Lights, s_Swerve, s_ShooterPivot, s_Eyes);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void initialize() {
        // Runs once when scheduled
        if (s_Swerve.redAlliance) {

            if ((Math.abs(s_Swerve.getHeading().getDegrees() - (s_Eyes.getTargetRotation() * 0.12)) < swerveTolerance)
                && (Math.abs(s_ShooterPivot.m_ShooterPivot.getPosition().getValueAsDouble() - s_ShooterPivot.getShooterAngle()) < hoodTolerance)) {
                    s_Lights.ReadyToFire();
            } else {
                s_Lights.Aim();
            }

        } else {

            if ((Math.abs(s_Swerve.getHeading().getDegrees() - (s_Eyes.getTargetRotation() - s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()) * (.12)) < swerveTolerance)
                && (Math.abs(s_ShooterPivot.m_ShooterPivot.getPosition().getValueAsDouble() - s_ShooterPivot.getShooterAngle()) < hoodTolerance)) {
                    s_Lights.ReadyToFire();
            } else {
                s_Lights.Aim();
            }
        }
        
    }

    @Override
    public void execute() {
        // Runs repeatedly while scheduled
    }

    @Override
    public void end(boolean interrupted) {
        // Runs once when the command ends
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
