package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.arm.GoToState;
import frc.robot.commands.arm.Manual;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.Outtake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.macros.PickupCone;
import frc.robot.commands.macros.PlaceCone;
import frc.robot.commands.macros.Protect;
import frc.robot.commands.wrist.GoToPosition;
import frc.robot.commands.wrist.ManualWrist;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.WristSubsystem.WristState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final POVButton inside = new POVButton(operator, 270);
    private final POVButton low = new POVButton(operator, 180);
    private final POVButton medium = new POVButton(operator, 90);
    private final POVButton high = new POVButton(operator, 0);

    public enum Location { LOW, MIDDLE, HIGH }

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final WristSubsystem s_Wrist = new WristSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // s_Arm.setDefaultCommand(
        //     new Manual(s_Arm, () -> operator.getRawAxis(translationAxis))
        // );

        // s_Wrist.setDefaultCommand(
        //     new ManualWrist(s_Wrist, () -> operator.getRawAxis(XboxController.Axis.kRightY.value))
        // );

        s_Intake.setDefaultCommand(
            new StopIntake(s_Intake)
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        inside.whileTrue(new GoToState(s_Arm, ArmState.INSIDE));
        low.whileTrue(new GoToState(s_Arm, ArmState.LOW));
        medium.whileTrue(new GoToState(s_Arm, ArmState.MEDIUM));
        high.whileTrue(new GoToState(s_Arm, ArmState.HIGH));

        new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(
            new GoToPosition(s_Wrist, WristState.OUT)
        );

        new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(
            new GoToPosition(s_Wrist, WristState.DOWN)
        );

        new JoystickButton(operator, XboxController.Button.kLeftBumper.value).whileTrue(
            // new Intake(s_Intake)
            new Protect(s_Arm, s_Wrist)
        );

        new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileTrue(
            new Outtake(s_Intake)
        );

        new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(
            new PickupCone(s_Arm, s_Wrist, s_Intake)
        );

        new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(
            new PlaceCone(s_Arm, s_Wrist, null)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new RunEventfulAuto(s_Swerve, s_Arm, s_Wrist, s_Intake);
    }
}
