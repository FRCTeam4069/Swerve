package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefualtDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final XboxController Controller1 = new XboxController(0);

       public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new DefualtDriveCommand(
                swerveSubsystem,
                () -> Controller1.getLeftX(),
                () -> Controller1.getLeftY(),
                () -> Controller1.getRightX(),
                () -> !Controller1.getAButton()));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Button(Controller1::getAButton).whenPressed(() -> swerveSubsystem.resetGyro());
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(DriveConstants.kDriveKinematics);

        // // 2. Generate trajectory
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(1, 0),
        //                 new Translation2d(1, -1)),
        //         new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        //         trajectoryConfig);

        // // 3. Define PID controllers for tracking trajectory
        // PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // // 4. Construct command to follow trajectory
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //         trajectory,
        //         swerveSubsystem::getPose,
        //         DriveConstants.kDriveKinematics,
        //         xController,
        //         yController,
        //         thetaController,
        //         swerveSubsystem::setModuleStates,
        //         swerveSubsystem);

        // // 5. Add some init and wrap-up, and return everything
        // return new SequentialCommandGroup(
        //         new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        //         swerveControllerCommand,
        //         new InstantCommand(() -> swerveSubsystem.stopModules()));
        return new InstantCommand();
    }
}