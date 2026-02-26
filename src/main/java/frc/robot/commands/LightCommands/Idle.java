package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.swervedrive.*;

public class Idle extends Command {
    Lights s_Lights;
    SwerveSubsystem s_Swerve;
  /** Creates a new MoveIntake. */
    public Idle(Lights s_Lights, SwerveSubsystem s_Swerve) {
        this.s_Lights = s_Lights;
        this.s_Swerve = s_Swerve;
        addRequirements(s_Lights, s_Swerve);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void initialize() {
        // Runs once when scheduled
        if (s_Swerve.redAlliance && (s_Swerve.getPose().getX() < 4.0)) {
            new BadBoys(s_Lights);
        } else if (s_Swerve.blueAlliance && (s_Swerve.getPose().getX() > 13.0)) {
            new BadBoys(s_Lights);
        } else {
            new InstantCommand(() -> s_Lights.Idle());
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
