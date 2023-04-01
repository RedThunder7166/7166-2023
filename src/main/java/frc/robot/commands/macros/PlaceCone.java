package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.Location;
import frc.robot.commands.arm.GoToState;
import frc.robot.commands.wrist.GoToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.WristSubsystem.WristState;

public class PlaceCone extends ParallelCommandGroup {
    public PlaceCone (ArmSubsystem arm, WristSubsystem wrist, Location location) {
        addCommands(
            new GoToState(arm, ArmState.HIGH),
            new GoToPosition(wrist, WristState.PLACE_HIGH)
        );
    }
}
