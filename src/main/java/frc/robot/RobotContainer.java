package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.arm.GoToState;
import frc.robot.commands.arm.ManualArm;
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
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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

    /* Operator Contorls */
    private final int wristManual = XboxController.Axis.kLeftY.value;
    private final int armManual = XboxController.Axis.kRightY.value;
    private final JoystickButton wristTest = new JoystickButton(operator, XboxController.Button.kA.value);
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton creepModeButton = new JoystickButton(driver, XboxController.Button.kY.value);
    /* Operator Buttons */
    private final JoystickButton normalIntake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton normalOuttake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final POVButton inside = new POVButton(operator, 270);// NOTE: This is the DPAD.
    private final POVButton low = new POVButton(operator, 180);
    private final POVButton medium = new POVButton(operator, 90);
    private final POVButton high = new POVButton(operator, 0);
    private final JoystickButton CubeLED = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton ConeLED = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton fastOutake = new JoystickButton(operator, XboxController.Button.kX.value);




/* Autonomous Decider */
    private final SendableChooser<String> autoDecider = new SendableChooser<>();
    public enum Location {
        LOW, MIDDLE, HIGH, NONE
    }

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final WristSubsystem s_Wrist = new WristSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final LedSubsystem s_LedSubsystem = new LedSubsystem();

    private final Command s_ManualWristCommand = new ManualWrist(s_Wrist, () -> operator.getRawAxis(wristManual));
    private final Command s_ManualArmCommand = new ManualArm(s_Arm, () -> -operator.getRawAxis(armManual));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        Shuffleboard.getTab("Autonomous").add(autoDecider);
        autoDecider.addOption("Test 1", "Test 1");
        autoDecider.addOption("TwoCubeRight", "TwoCubeRight");

        CameraServer.startAutomaticCapture();

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()

                ));
        /* Manual modes */
        // s_Wrist.setDefaultCommand(new ManualWrist(s_Wrist, () -> operator.getRawAxis(wristManual)));
        // s_Arm.setDefaultCommand(new ManualArm(s_Arm, () -> -operator.getRawAxis(armManual)));

        s_Intake.setDefaultCommand(
                new StopIntake(s_Intake));

        // Configure the button bindings
        configureButtonBindings();
    }

    public void controllerLoop(){
        double wristAxis = operator.getRawAxis(wristManual);
        if (Math.abs(wristAxis) > 0.25) {
                s_ManualWristCommand.schedule();
        } else if (s_ManualWristCommand.isScheduled()) {
                s_ManualWristCommand.cancel();
                s_Wrist.stopMotor();
        }

        double armAxis = operator.getRawAxis(armManual);
        if (Math.abs(armAxis) > 0.25) {
                s_ManualArmCommand.schedule();
        } else if (s_ManualArmCommand.isScheduled()) {
                s_ManualArmCommand.cancel();
                s_Arm.stopMotor();
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        inside.onTrue(new SequentialCommandGroup(
              //  new GoToPosition(s_Wrist, 20),
                new GoToState(s_Arm, ArmState.INSIDE)));

        low.onTrue(new SequentialCommandGroup(
              //  new GoToPosition(s_Wrist, 20),
                new GoToState(s_Arm, ArmState.LOW)));

        medium.onTrue(new SequentialCommandGroup(
              // new GoToPosition(s_Wrist, 20),
                new GoToState(s_Arm, ArmState.MEDIUM)
        ));

        high.onTrue(new SequentialCommandGroup(
                new GoToState(s_Arm, ArmState.HIGH),
                new GoToPosition(s_Wrist, 180)
        ));


        wristTest.onTrue(new GoToPosition(s_Wrist, 20.0));
                        // new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(
        // new GoToPosition(s_Wrist, WristState.OUT)
        // );

        // new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(
        // new GoToPosition(s_Wrist, WristState.DOWN)
        // );

        normalIntake.whileTrue(
                new Intake(s_Intake));

        normalOuttake.whileTrue(
                new Outtake(s_Intake));

        ConeLED.onTrue(new InstantCommand(() -> s_LedSubsystem.setBoth(1), s_LedSubsystem));
        CubeLED.onTrue(new InstantCommand(() -> s_LedSubsystem.setBoth(0), s_LedSubsystem));

        fastOutake.whileTrue(new RunCommand(()-> {
                s_Intake.fastOutake();
        }));
        // new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(
        // new PickupCone(s_Arm, s_Wrist, s_Intake)
        // );

        // new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(
        // new PlaceCone(s_Arm, s_Wrist, null)
        // );

        creepModeButton.onTrue(new InstantCommand(() -> s_Swerve.enableCreepMode()));
        creepModeButton.onFalse(new InstantCommand(() -> s_Swerve.disableCreepMode()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new RunEventfulAuto(s_Swerve, s_Arm, s_Wrist, s_Intake, autoDecider.getSelected());
    }

    public void robotInit() {
        s_Arm.stopMotor();
        s_Wrist.stopMotor();
    }

    public void disabledInit() {
        s_LedSubsystem.setLeftBlinkIn(-0.5);
        s_LedSubsystem.setRightBlinkIn(0.5);
    }
}
