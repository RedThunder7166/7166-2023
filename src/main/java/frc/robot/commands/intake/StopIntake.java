package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntake extends CommandBase{
    private IntakeSubsystem intake;
    public StopIntake (IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // intake.setPower(-0.08);
        intake.setPower(0.0);
    }
}
