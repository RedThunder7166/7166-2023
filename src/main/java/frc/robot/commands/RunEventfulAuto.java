package frc.robot.commands;


import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.arm.GoToState;
import frc.robot.commands.intake.FastOuttake;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.Outtake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.wrist.GoToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.WristSubsystem.WristState;

public class RunEventfulAuto extends SequentialCommandGroup {
    public static HashMap<String, Command> eventMap = new HashMap<>();
    public RunEventfulAuto (Swerve swerve, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake, String PathName) {
        List<PathPlannerTrajectory> group = PathPlanner.loadPathGroup(PathName, new PathConstraints(3, 3));

        eventMap.put("ARM_CUBE_MED", new GoToState(arm, 56));

        eventMap.put("ARM_IN", new GoToState(arm, ArmState.INSIDE));
        eventMap.put("ARM_LOW", new GoToState(arm, ArmState.LOW));
        eventMap.put("ARM_MEDIUM", new GoToState(arm, ArmState.MEDIUM));
        eventMap.put("ARM_HIGH", new GoToState(arm, ArmState.HIGH));
        eventMap.put("ARM_CUBE_HIGH", new GoToState(arm, 87));
        eventMap.put("WRIST_CUBE_HIGH", new GoToPosition(wrist, 60));
        eventMap.put("WRIST_TRANSPORT", new GoToPosition(wrist, WristState.TRANSPORT));
        eventMap.put("WRIST_LOW", new GoToPosition(wrist, WristState.LOW));
        eventMap.put("WRIST_PLACE_MID", new GoToPosition(wrist, WristState.PLACE_MID));
        eventMap.put("WRIST_PLACE_HIGH", new GoToPosition(wrist, WristState.PLACE_HIGH));
        eventMap.put("WRIST_GROUND_CONE", new GoToPosition(wrist, WristState.GROUND_CONE));
        eventMap.put("BALANCE_AUTO", new AutoBalance(swerve));
        eventMap.put("INTAKE", new Intake(intake).raceWith(new WaitCommand(.1)));
        eventMap.put("LONG_INTAKE", new Intake(intake).raceWith(new WaitCommand(1)));

        eventMap.put("OUTTAKE", new Outtake(intake).raceWith(new WaitCommand(.4)));
        eventMap.put("LONG_OUTTAKE", new Outtake(intake).raceWith(new WaitCommand(1)));
        eventMap.put("FAST_OUTTAKE", new FastOuttake(intake).raceWith(new WaitCommand(.2)));
        eventMap.put("STOP_INTAKE", new StopIntake(intake));

        // eventMap.put("score-cube", new SequentialCommandGroup(
        //     new ParallelCommandGroup(
        //         new GoToState(arm, ArmState.MEDIUM),
        //         new GoToPosition(wrist, WristState.PLACE_MID)
        //     ),
        //     new Outtake(intake)
        // ));

        eventMap.put("TEST1", new PrintCommand("REACHED TEST 1 LETS GOOOOOOOOOOOOO"));

        addCommands(
            new SwerveAutoBuilder(
                swerve::getPose,
                swerve::resetOdometry,
                Constants.Swerve.swerveKinematics,
                new PIDConstants(5.0, 0, 0), 
                new PIDConstants(3.0, 0, 0),
                swerve::setModuleStates,
                eventMap,
                true,
                swerve
            ).fullAuto(group)
        );
    }
}
