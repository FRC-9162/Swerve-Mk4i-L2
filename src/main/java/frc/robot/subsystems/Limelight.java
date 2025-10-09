package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    NetworkTable Limelight;

    public Limelight(){

        Limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

    }

    public double getTV(){
        return Limelight.getEntry("tv").getDouble(0.0);

    }

    public double getTA(){
        return Limelight.getEntry("ta").getDouble(0.0);
    }

    public double getTX(){
        return Limelight.getEntry("tx").getDouble(0.0);
    }

    public int getID(){
        return (int) Limelight.getEntry("tid").getDouble(-1);
    }
    
}