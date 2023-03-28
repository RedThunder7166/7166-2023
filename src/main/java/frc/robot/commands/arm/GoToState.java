package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class GoToState extends CommandBase {
    private ArmSubsystem arm;
    private double position;

    public GoToState (ArmSubsystem arm, double position) {
        this.arm = arm;
        addRequirements(arm);

        this.position = position;
    }

    public GoToState (ArmSubsystem arm, ArmState state) {
        this.arm = arm;
        addRequirements(arm);

        switch (state) {
            case INSIDE:
                this.position = 0.0;
                break;

            case LOW:
                this.position = 30.0;
                break;

            case MEDIUM:
                this.position = 70.0;
                break;

            case HIGH:
                this.position = 100.0;
                break;
        
            default:
                break;
        }
    }

    @Override
    public void execute() {
        arm.setAngle(position);
    }

    @Override
    public boolean isFinished() {
        return (arm.getAngle() < position + 1.5 && arm.getAngle() > position - 1.5);
    }
}
