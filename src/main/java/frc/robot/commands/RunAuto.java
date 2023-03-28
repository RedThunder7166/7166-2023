package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class RunAuto extends SequentialCommandGroup {
    public RunAuto (Swerve swerve) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Test 2", new PathConstraints(4, 3));

        addCommands(
            new InstantCommand(() -> {
                swerve.resetOdometry(trajectory.getInitialHolonomicPose());
            }),
            new PPSwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(5.0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(5.0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(2.0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                swerve::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                swerve
            )
        );
    }
}
