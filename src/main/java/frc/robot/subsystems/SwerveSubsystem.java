package frc.robot.subsystems;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase{

    private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

    private final Module FRSwerveModule = new Module(
        DrivebaseConstants.FR_DRIVE_MOTOR,
        DrivebaseConstants.FR_STEER_MOTOR,
        DrivebaseConstants.FR_DRIVE_MOTOR_REVERSED, 
        DrivebaseConstants.FR_STEER_MOTOR_REVERSED,
        DrivebaseConstants.FR_STEER_ENCODER,
        DrivebaseConstants.FR_STEER_OFFSET,
        DrivebaseConstants.FR_STEER_ENCODER_REVERSED
    );

    private final Module FLSwerveModule = new Module(
        DrivebaseConstants.FL_DRIVE_MOTOR,
        DrivebaseConstants.FL_STEER_MOTOR,
        DrivebaseConstants.FL_DRIVE_MOTOR_REVERSED, 
        DrivebaseConstants.FL_STEER_MOTOR_REVERSED,
        DrivebaseConstants.FL_STEER_ENCODER,
        DrivebaseConstants.FL_STEER_OFFSET,
        DrivebaseConstants.FL_STEER_ENCODER_REVERSED
    );

    private final Module BRSwerveModule = new Module(
        DrivebaseConstants.BR_DRIVE_MOTOR,
        DrivebaseConstants.BR_STEER_MOTOR,
        DrivebaseConstants.BR_DRIVE_MOTOR_REVERSED, 
        DrivebaseConstants.BR_STEER_MOTOR_REVERSED,
        DrivebaseConstants.BR_STEER_ENCODER,
        DrivebaseConstants.BR_STEER_OFFSET,
        DrivebaseConstants.BR_STEER_ENCODER_REVERSED
    );

    private final Module BLSwerveModule = new Module(
        DrivebaseConstants.BL_DRIVE_MOTOR,
        DrivebaseConstants.BL_STEER_MOTOR,
        DrivebaseConstants.BL_DRIVE_MOTOR_REVERSED, 
        DrivebaseConstants.BL_STEER_MOTOR_REVERSED,
        DrivebaseConstants.BL_STEER_ENCODER,
        DrivebaseConstants.BL_STEER_OFFSET,
        DrivebaseConstants.BL_STEER_ENCODER_REVERSED
    );
    
    private Pigeon2 gyro = new Pigeon2(DrivebaseConstants.PIGEON_ID);


    public SwerveSubsystem(){
       new Thread(() -> {
        try{
            Thread.sleep(1000);
            resetGyro();
        } catch(Exception e){
        } 
    }).start();
        
    
    SmartDashboard.putNumber("Front Right Wheel Radains", DrivebaseConstants.FR_STEER_OFFSET);
    SmartDashboard.putNumber("Front Left Wheel Radains", DrivebaseConstants.FR_STEER_OFFSET);
    SmartDashboard.putNumber("Back Right Wheel Radains", DrivebaseConstants.BR_STEER_OFFSET);
    SmartDashboard.putNumber("Back Left Wheel Radains", DrivebaseConstants.BL_STEER_OFFSET);

    }
    public void resetGyro(){
        gyro.setYaw(0);
    }

    public double getHeading(){
        return gyro.getYaw() % 360;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules(){
        FRSwerveModule.stop();
        FLSwerveModule.stop();
        BRSwerveModule.stop();
        BLSwerveModule.stop();
    }

    public void setModuleStates(SwerveModuleState[] ModuleStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(ModuleStates, ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);
        FLSwerveModule.setDesiredState(ModuleStates[0]);
        FRSwerveModule.setDesiredState(ModuleStates[1]);;
        BLSwerveModule.setDesiredState(ModuleStates[2]);
        BRSwerveModule.setDesiredState(ModuleStates[3]);

        SmartDashboard.putString("Module State FL", ModuleStates[0].toString());
        SmartDashboard.putString("Module State FR", ModuleStates[1].toString());
        SmartDashboard.putString("Module State BL", ModuleStates[2].toString());
        SmartDashboard.putString("Module State BR", ModuleStates[3].toString());

    }

    public void printNumbers(){
        SmartDashboard.putNumber("FL Rad", FLSwerveModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FR Rad", FRSwerveModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BF Rad", BLSwerveModule.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BR Rad", BRSwerveModule.getAbsoluteEncoderRad());

        SmartDashboard.putNumber("FL Voltage", FLSwerveModule.getDriveVoltage());
        SmartDashboard.putNumber("FR Voltage", FRSwerveModule.getDriveVoltage());
        SmartDashboard.putNumber("BF Voltage", BLSwerveModule.getDriveVoltage());
        SmartDashboard.putNumber("BR Voltage", BRSwerveModule.getDriveVoltage());
    }

    public void EditOffsets(){

        // ShuffleboardTab OffsetEditor = Shuffleboard.getTab("Offset Editor");
        // Shuffleboard.getTab(OffsetEditor.getTitle())
        // .add("Front Right Wheel Radains", DrivebaseConstants.FR_STEER_OFFSET)
        // .add("Front Left Wheel Radains", DrivebaseConstants.FR_STEER_OFFSET)
        // .add("Back Right Wheel Radains", DrivebaseConstants.BR_STEER_OFFSET)
        // .add("Back Left Wheel Radains", DrivebaseConstants.BL_STEER_OFFSET);    

       DrivebaseConstants.FR_STEER_OFFSET =  SmartDashboard.getNumber("Front Right Wheel Radains", DrivebaseConstants.FR_STEER_OFFSET);
       DrivebaseConstants.FL_STEER_OFFSET =  SmartDashboard.getNumber("Front Left Wheel Radains", DrivebaseConstants.FL_STEER_OFFSET);
       DrivebaseConstants.BR_STEER_OFFSET =  SmartDashboard.getNumber("Back Right Wheel Radains", DrivebaseConstants.BR_STEER_OFFSET);
       DrivebaseConstants.BL_STEER_OFFSET =  SmartDashboard.getNumber("Back Left Wheel Radains", DrivebaseConstants.BL_STEER_OFFSET);

    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        if(Constants.PrintDebugNumbers) {printNumbers(); SmartDashboard.delete("Print Debug Number?");}
        else{ 
            SmartDashboard.putBoolean("Print Debug Number?", false);
            Constants.PrintDebugNumbers = SmartDashboard.getBoolean("Print Debug Number?", false);
        }
        if(Constants.OffsetTuning){ EditOffsets(); SmartDashboard.delete("Turn on Offset Turning?"); }
        else{ 
            SmartDashboard.putBoolean("Turn on Offset Turning?", false);
            Constants.OffsetTuning = SmartDashboard.getBoolean("Turn on Offset Turning?", false);
        }

    }

}
