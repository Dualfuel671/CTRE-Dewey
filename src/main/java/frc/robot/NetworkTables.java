package frc.robot;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        final NetworkTable table = inst.getTable("Laptop");
        BooleanSubscriber testBoolean;

    public NetworkTables(){

        testBoolean = table.getBooleanTopic("testBoolean").subscribe(false);
    }


    //public boolean getTestEntry(){
      //  return testBoolean.get();
    //}
}
