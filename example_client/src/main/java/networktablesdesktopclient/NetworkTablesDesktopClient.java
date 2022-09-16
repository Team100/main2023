package networktablesdesktopclient;

import edu.wpi.first.networktables.EntryInfo;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTablesDesktopClient {
  public static void main(String[] args) {
    new NetworkTablesDesktopClient().run();
  }

  public void run() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startClient("wpilibpi.local");
    NetworkTable table = inst.getTable("Vision");
    NetworkTableEntry idEntry = table.getEntry("id");
    while (true) {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException ex) {
        System.out.println("interrupted");
        return;
      }
      for(double i: idEntry.getDoubleArray(new double[]{0})) {
        System.out.println("id: " + i);
      }
    }
  }
}
