package frc.robot.commands;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.GoToState;
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
    public RunEventfulAuto (Swerve swerve, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake, String PathName) {
        List<PathPlannerTrajectory> group = PathPlanner.loadPathGroup(PathName, new PathConstraints(4, 3));

        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("Outake", new Outtake(intake));
        eventMap.put("ARM_CUBE_MED", new GoToState(arm, 56));
        eventMap.put("ARM_IN", new GoToState(arm,ArmState.INSIDE ));
        eventMap.put("ARM_LOW", new GoToState(arm,ArmState.LOW));
         eventMap.put("Intake", new ParallelRaceGroup(
            new Intake(intake),
            new WaitCommand(1)));
          eventMap.put("STOP_INTAKE", new StopIntake(intake));
         eventMap.put("OUTAKE", new ParallelRaceGroup(
            new Outtake(intake),
            new WaitCommand(1)
         ));
        // eventMap.put("start-intake", new Intake(intake));
        // eventMap.put("stop-intake", new StopIntake(intake));
        // eventMap.put("protect", new ParallelCommandGroup(
            // new GoToState(arm, ArmState.INSIDE),
            // new GoToPosition(wrist, WristState.TRANSPORT)
        // ));
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
