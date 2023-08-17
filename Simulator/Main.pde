String Mode = "POWER_ON";
/* CHARGING STOP PAUSE START RUNNING POWER_ON*/

void MainCharging(){}
void MainStop(){}
void MainPause(){}
void MainStart(){}
void MainRunning(){}
void MainPowerOn(){
  LoadConfiguration();
  GenerateGcode(); //TODO: REMOVE
}

void QueueMainBluetooth(){}
