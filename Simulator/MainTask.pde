String Mode = "POWER_ON";
/* CHARGING STOP PAUSE START RUNNING POWER_ON SETUP*/

void MainCharging(){}
void MainChargingStart(){
  println("Charging start");
}
void MainChargingStop(){}
void MainStop(){println("Strayed from path (STOP)");}
void MainPause(){}
void MainStart(){}
void MainRunning(){}
void MainPowerOn(){
  LoadConfiguration();
  GenerateGcode();
}
void MainSetup(){}

void QueueMainBluetooth(){}
