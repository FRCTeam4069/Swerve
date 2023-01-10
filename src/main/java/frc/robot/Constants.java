// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static boolean PrintDebugNumbers = true; // prints Debug info for wheels and speeds 
    public static boolean OffsetTuning = true; // Enable if you would like to tune the turning encoder offsets 
    
    public static final class SparkMaxVoltage{
        public static final int DriveingMotorVoltage = 20;
        public static final int TurningMotorVoltage = 20;
    }

    public static final class DrivebaseConstants{

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(9.39)*2; 
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(9.39)*2; 
    
        public static final int PIGEON_ID = 17;

        public static final int FL_DRIVE_MOTOR = 5;
        public static final int FL_STEER_MOTOR = 1; 
        public static final boolean FL_DRIVE_MOTOR_REVERSED = false;                        //FIXME
        public static final boolean FL_STEER_MOTOR_REVERSED = true;                        //FIXME
        public static final int FL_STEER_ENCODER = 10; 
        public static double FL_STEER_OFFSET = 0 ; //-2         //FIXME
        public static final boolean FL_STEER_ENCODER_REVERSED = true;                      //FIXME
    
        public static final int FR_DRIVE_MOTOR = 6; 
        public static final int FR_STEER_MOTOR = 2;
        public static final boolean FR_DRIVE_MOTOR_REVERSED = false;                        //FIXME
        public static final boolean FR_STEER_MOTOR_REVERSED = true;                        //FIXME
        public static final int FR_STEER_ENCODER = 9; 
        public static double FR_STEER_OFFSET = 0; //-2         //FIXME
        public static final boolean FR_STEER_ENCODER_REVERSED = true;                      //FIXME
    
        public static final int BL_DRIVE_MOTOR = 8; 
        public static final int BL_STEER_MOTOR = 4; 
        public static final boolean BL_DRIVE_MOTOR_REVERSED = false;                        //FIXME
        public static final boolean BL_STEER_MOTOR_REVERSED = true;                        //FIXME
        public static final int BL_STEER_ENCODER = 12;
        public static double BL_STEER_OFFSET = 0; //+4         //FIXME
        public static final boolean BL_STEER_ENCODER_REVERSED = true;                      //FIXME
    
        public static final int BR_DRIVE_MOTOR =7; 
        public static final int BR_STEER_MOTOR = 3; 
        public static final boolean BR_DRIVE_MOTOR_REVERSED = false;                        //FIXME
        public static final boolean BR_STEER_MOTOR_REVERSED = true;                        //FIXME
        public static final int BR_STEER_ENCODER = 11;         
        public static double BR_STEER_OFFSET = 0 ; //+2         //FIXME
        public static final boolean BR_STEER_ENCODER_REVERSED = true;                      //FIXME

        
        public static final double xSpeedSlewRate = 3;
        public static final double ySpeedSlewRate = 3;
        public static final double TurnSpeedSlewRate = 3;

        public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    }

    public static final class ModuleConstants{
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = SdsModuleConfigurations.MK4I_L2.getDriveReduction() ;
    public static final double kTurningMotorGearRatio = SdsModuleConfigurations.MK4I_L2.getSteerReduction();
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
    }

    public static final class InputConstants{

        public static final double Contoller1 = 0;


    }

}
