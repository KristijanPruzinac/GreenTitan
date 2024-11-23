#include "Configuration.h"

#define FORMAT_LITTLEFS_IF_FAILED true

FileResult listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
    File root = fs.open(dirname);
    if (!root) {
        return FAILED_OPEN;
    }
    if (!root.isDirectory()) {
        return NOT_A_DIRECTORY;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            FileResult result = listDir(fs, file.path(), levels - 1);
            if (result != SUCCESS) {
                return result;
            }
        }
        file = root.openNextFile();
    }

    return SUCCESS;
}

FileResult createDir(fs::FS &fs, const char *path) {
    if (fs.mkdir(path)) {
        return SUCCESS;
    } else {
        return MKDIR_FAILED;
    }
}

FileResult removeDir(fs::FS &fs, const char *path) {
    if (fs.rmdir(path)) {
        return SUCCESS;
    } else {
        return RMDIR_FAILED;
    }
}

FileResult readFile(fs::FS &fs, const char *path, String &fileContent) {
    File file = fs.open(path);
    if (!file || file.isDirectory()) {
        return FAILED_OPEN;
    }

    while (file.available()) {
        char c = file.read();
        fileContent += c;
    }
    file.close();

    return SUCCESS;
}

FileResult writeFile(fs::FS &fs, const char *path, const char *message) {
    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        return FAILED_OPEN;
    }
    if (file.print(message)) {
        file.close();
        return SUCCESS;
    } else {
        file.close();
        return FAILED_WRITE;
    }
}

FileResult appendFile(fs::FS &fs, const char *path, const char *message) {
    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        return FAILED_OPEN;
    }
    if (file.print(message)) {
        file.close();
        return SUCCESS;
    } else {
        file.close();
        return APPEND_FAILED;
    }
}

FileResult renameFile(fs::FS &fs, const char *path1, const char *path2) {
    if (fs.rename(path1, path2)) {
        return SUCCESS;
    } else {
        return RENAME_FAILED;
    }
}

FileResult deleteFile(fs::FS &fs, const char *path) {
    if (fs.remove(path)) {
        return SUCCESS;
    } else {
        return DELETE_FAILED;
    }
}

FileResult InitConfiguration(){
  if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
    return INIT_FAILED;
  }

  FileResult configurationResult = LoadConfiguration();

  return configurationResult;
}

FileResult SaveConfiguration() {
    String ConfigWriteString = "";
    ConfigWriteString += "SETUP_COMPLETED " + String(SETUP_COMPLETED) + "\n";
    ConfigWriteString += "MOWER_OVERLAP " + String(MOWER_OVERLAP) + "\n";
    ConfigWriteString += "MAX_DEVIATION " + String(MAX_DEVIATION) + "\n";
    ConfigWriteString += "BASE_LON " + String(BASE_LON) + "\n";
    ConfigWriteString += "BASE_LAT " + String(BASE_LAT) + "\n";
    ConfigWriteString += "BASE_EXIT_LON " + String(BASE_EXIT_LON) + "\n";
    ConfigWriteString += "BASE_EXIT_LAT " + String(BASE_EXIT_LAT) + "\n";
    ConfigWriteString += "GPS_ACC_THRESHOLD " + String(GPS_ACC_THRESHOLD) + "\n";
    ConfigWriteString += "GPS_STABILITY_CHECK_DURATION " + String(GPS_STABILITY_CHECK_DURATION) + "\n";
    ConfigWriteString += "BATTERY_LEVEL_MIN " + String(BATTERY_LEVEL_MIN) + "\n";
    ConfigWriteString += "BATTERY_LEVEL_MAX " + String(BATTERY_LEVEL_MAX) + "\n";
    ConfigWriteString += "MOTOR_SIDE_INVERT " + String(MOTOR_SIDE_INVERT) + "\n";
    ConfigWriteString += "MOTOR_LEFT_INVERT " + String(MOTOR_LEFT_INVERT) + "\n";
    ConfigWriteString += "MOTOR_RIGHT_INVERT " + String(MOTOR_RIGHT_INVERT) + "\n";
    ConfigWriteString += "MOTOR_OPTIMAL_VOLTAGE " + String(MOTOR_OPTIMAL_VOLTAGE) + "\n";
    
    //Config status
    ConfigWriteString += "CONFIG_PATH " + String(CONFIG_PATH) + "\n";
    ConfigWriteString += "CONFIG_BATTERY " + String(CONFIG_BATTERY) + "\n";
    ConfigWriteString += "CONFIG_MOTORS " + String(CONFIG_MOTORS) + "\n";
    ConfigWriteString += "CONFIG_GYRO " + String(CONFIG_GYRO) + "\n";
    ConfigWriteString += "CONFIG_RAIN_SENSOR " + String(CONFIG_RAIN_SENSOR) + "\n";

    //Outlines
    ConfigWriteString += "ALGORITHM_PATH\n";
    ConfigWriteString += AlgorithmGetPathString();

    deleteFile(LittleFS, "/Configuration.txt");
    return writeFile(LittleFS, "/Configuration.txt", ConfigWriteString.c_str());
}
FileResult LoadConfiguration() {
    String readData = "";
    FileResult result = readFile(LittleFS, "/Configuration.txt", readData);
    if (result != SUCCESS || readData.length() == 0) {
        return result;
    }

    // Split readData by newline character
    int newlineIndex = 0;
    while ((newlineIndex = readData.indexOf('\n')) != -1) {
        String line = readData.substring(0, newlineIndex);
        readData = readData.substring(newlineIndex + 1);

        // Split each line by space and assign values to variables
        int spaceIndex = line.indexOf(' ');
        if (spaceIndex != -1) {
            String variableName = line.substring(0, spaceIndex);
            String variableValue = line.substring(spaceIndex + 1);

            // Convert variableValue to the appropriate type (int or float)
            if (variableName == "SETUP_COMPLETED") {
                SETUP_COMPLETED = variableValue.toInt();
            } else if (variableName == "MOWER_OVERLAP") {
                MOWER_OVERLAP = variableValue.toInt();
            } else if (variableName == "MAX_DEVIATION") {
                MAX_DEVIATION = variableValue.toInt();
            } else if (variableName == "BASE_LON") {
                BASE_LON = variableValue.toInt();
            } else if (variableName == "BASE_LAT") {
                BASE_LAT = variableValue.toInt();
            } else if (variableName == "BASE_EXIT_LON") {
                BASE_EXIT_LON = variableValue.toInt();
            } else if (variableName == "BASE_EXIT_LAT") {
                BASE_EXIT_LAT = variableValue.toInt();
            } else if (variableName == "GPS_ACC_THRESHOLD"){
                GPS_ACC_THRESHOLD = variableValue.toInt();
            } else if (variableName == "GPS_STABILITY_CHECK_DURATION"){
                GPS_STABILITY_CHECK_DURATION = variableValue.toInt();
            } else if (variableName == "BATTERY_LEVEL_MIN") {
                BATTERY_LEVEL_MIN = variableValue.toFloat();
            } else if (variableName == "BATTERY_LEVEL_MAX") {
                BATTERY_LEVEL_MAX = variableValue.toFloat();
            } else if (variableName == "MOTOR_SIDE_INVERT") {
                MOTOR_SIDE_INVERT = (variableValue.toInt() != 0);
            } else if (variableName == "MOTOR_LEFT_INVERT") {
                MOTOR_LEFT_INVERT = (variableValue.toInt() != 0);
            } else if (variableName == "MOTOR_RIGHT_INVERT") {
                MOTOR_RIGHT_INVERT = (variableValue.toInt() != 0);
            } else if (variableName == "MOTOR_OPTIMAL_VOLTAGE") {
                MOTOR_OPTIMAL_VOLTAGE = variableValue.toFloat();
            }
        }
        else if (line == "ALGORITHM_PATH") {
          if (AlgorithmPopulatePathFromString(readData)){
            return SUCCESS;
          }
          else {
            return ALGORITHM_FAILED;
          }
        }
    }

    return SUCCESS;
}