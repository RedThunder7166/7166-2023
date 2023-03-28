package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.GoToState;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.wrist.GoToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.WristSubsystem.WristState;

public class PickupCone extends ParallelCommandGroup {
    public PickupCone (ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake) {
        // addRequirements(arm, wrist, intake);
        addCommands(
            new GoToState(arm, ArmState.LOW),
            new GoToPosition(wrist, WristState.HALFWAY),
            new Intake(intake)
        );
    }
}
