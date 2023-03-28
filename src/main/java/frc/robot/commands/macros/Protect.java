package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.GoToState;
import frc.robot.commands.wrist.GoToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.WristSubsystem.WristState;

public class Protect extends ParallelCommandGroup {
    public Protect (ArmSubsystem arm, WristSubsystem wrist) {
        addCommands(
            new GoToState(arm, ArmState.INSIDE),
            new GoToPosition(wrist, WristState.HALFWAY)
        );
    }
}
