package frc.robot.commands.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class ManualWrist extends CommandBase {
    private WristSubsystem wrist;
    private DoubleSupplier power;

    public ManualWrist (WristSubsystem wrist, DoubleSupplier power) {
        this.wrist = wrist;
        addRequirements(wrist);

        this.power = power;
    }

    @Override
    public void execute() {
        wrist.manualControl(power.getAsDouble() * 0.2);
    }
}
