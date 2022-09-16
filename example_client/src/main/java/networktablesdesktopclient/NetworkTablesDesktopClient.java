package networktablesdesktopclient;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTablesDesktopClient {
  public static void main(String[] args) {
    new NetworkTablesDesktopClient().run();
  }

  public void run() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Vision");
    NetworkTableEntry idEntry = table.getEntry("id");
    //inst.startClientTeam(294);
    inst.startClient("wpilibpi.local");
    while (true) {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException ex) {
        System.out.println("interrupted");
        return;
      }
      double id = idEntry.getDouble(0.0);
      System.out.println("id: " + id);
    }
  }
}
