package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DefualtDriveCommand extends CommandBase{
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpd, ySpd, TSpd;
    private final Supplier<Boolean> FieldOriented;
    private final SlewRateLimiter xSlewRateLimiter, ySlewRateLimiter, turnSlewRateLimiter;

    public DefualtDriveCommand(SwerveSubsystem swerveSubsystem, 
    Supplier<Double> xSpd, Supplier<Double> ySpd, Supplier<Double> TSpd, 
    Supplier<Boolean> FieldOriented) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpd = xSpd;
        this.ySpd = ySpd;
        this.TSpd = TSpd;
        this.FieldOriented = FieldOriented;
        this.xSlewRateLimiter = new SlewRateLimiter(DrivebaseConstants.xSpeedSlewRate);
        this.ySlewRateLimiter = new SlewRateLimiter(DrivebaseConstants.ySpeedSlewRate);
        this.turnSlewRateLimiter = new SlewRateLimiter(DrivebaseConstants.TurnSpeedSlewRate);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        double xspeed = xSpd.get();
        double yspeed = ySpd.get();
        double Tspeed = TSpd.get();

        xspeed = Math.abs(xspeed)>0.01? xspeed : 0;
        yspeed = Math.abs(yspeed)>0.01? yspeed : 0;
        Tspeed = Math.abs(Tspeed)>0.01? Tspeed : 0;

        xspeed = xSlewRateLimiter.calculate(xspeed) * ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
        yspeed = ySlewRateLimiter.calculate(yspeed) *  ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
        Tspeed = turnSlewRateLimiter.calculate(Tspeed) *  ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;

        ChassisSpeeds chassisSpeeds;

        if (FieldOriented.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xspeed, yspeed, Tspeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xspeed , yspeed, Tspeed);
        }

        //  Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = DrivebaseConstants.m_kinematics.toSwerveModuleStates(chassisSpeeds);

        //  Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
