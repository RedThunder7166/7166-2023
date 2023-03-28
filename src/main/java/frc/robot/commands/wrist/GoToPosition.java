package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.WristState;

public class GoToPosition extends CommandBase {
    private WristSubsystem wrist;
    private double position;

    public GoToPosition (WristSubsystem wrist, double position) {
        this.wrist = wrist;
        addRequirements(wrist);

        this.position = position;
    }

    public GoToPosition (WristSubsystem wrist, WristState state) {
        this.wrist = wrist;
        addRequirements(wrist);

        switch (state) {
            case DOWN:
                this.position = 100.0;
                break;

            case PLACING:
                this.position = 80.0;
                break;

            case HALFWAY:
                this.position = 50.0;
                break;

            case OUT:
                this.position = 25.0;
                break;
        
            default:
                break;
        }
    }

    @Override
    public void execute() {
        wrist.setAngle(position);
    }

    @Override
    public boolean isFinished() {
        return (wrist.getAngle() < position + 2.0 && wrist.getAngle() > position - 2.0 );
    }
}
