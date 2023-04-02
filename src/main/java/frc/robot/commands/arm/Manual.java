package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class Manual extends CommandBase {
    private ArmSubsystem arm;
    private DoubleSupplier power;

    public Manual (ArmSubsystem arm, DoubleSupplier power) {
        this.arm = arm;
        addRequirements(arm);

        this.power = power;
    }

    @Override
    public void execute() {
        this.arm.manualControl(power.getAsDouble() * 0.4);
    }
}
