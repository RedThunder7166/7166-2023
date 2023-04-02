package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.WristState;

public class GoToPosition extends CommandBase {
    private WristSubsystem wrist;
    private double position;

    public GoToPosition(WristSubsystem wrist, double position) {
        this.wrist = wrist;
        addRequirements(wrist);

        this.position = position;
    }

    public GoToPosition(WristSubsystem wrist, WristState state) {// TODO update values to negatives
        this.wrist = wrist;
        addRequirements(wrist);

        switch (state) {
            case TRANSPORT:
                this.position = 20.0;
                break;

            case LOW:
                this.position = 77.0;
                break;

            case PLACE_MID:
                this.position = 155.0;
                break;

            case PLACE_HIGH:
                this.position = 185.0;
                break;

            case GROUND_CONE:
                this.position = 176;
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
        return (wrist.getAngle() < position + 2.0 && wrist.getAngle() > position - 2.0);
    }
}
