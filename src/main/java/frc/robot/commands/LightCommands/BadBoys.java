package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Lights;

public class BadBoys extends Command {
    Lights s_Lights;
  /** Creates a new MoveIntake. */
    public BadBoys(Lights s_Lights) {
        this.s_Lights = s_Lights;
        addRequirements(s_Lights);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void initialize() {
        // Runs once when scheduled
    }

    @Override
    public void execute() {
        // Runs repeatedly while scheduled
        new SequentialCommandGroup(
            new InstantCommand(() -> s_Lights.Defense()),
            new WaitCommand(0.5),
            new InstantCommand(() -> s_Lights.DefenseAlternate()),
            new WaitCommand(0.5)
        );
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
