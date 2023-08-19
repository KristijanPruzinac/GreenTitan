//Main task

//ALGORITHM SETTINGS
int MOWER_OVERLAP = 85;

void SaveConfiguration(){}
void LoadConfiguration(){

  //TODO: READ CONFIGURATION FILE (LittleFS)
  String ConfigurationData;
  
  //TODO: REMOVE SIMULATOR DATA
  ConfigurationData = new String(loadBytes("config.txt"));
  
  //Parse data
  String[] lines = splitStringByCharacters(ConfigurationData, "\n");
  
  int lineIndex = 0;
  while (lineIndex < lines.length){
    //OUTLINE START
    if (lines[lineIndex].charAt(0) == 'O'){
      outlines.add(new ArrayList<ArrayList<Long>>());
    }
    //GCODE START
    else if (lines[lineIndex].charAt(0) == 'G'){
      //TODO: READ GCODE
    }
    else {
      String[] dataXY = splitStringByCharacters(lines[lineIndex], " ");
      
      ArrayList<ArrayList<Long>> outline = outlines.get(outlines.size() - 1);
      
      outline.add(new ArrayList<Long>());
      outline.get(outline.size() - 1).add(Long.parseLong((dataXY[0].trim()))); //WARNING: USE .toInt() in arduino
      outline.get(outline.size() - 1).add(Long.parseLong((dataXY[1].trim())));
    }
    
    lineIndex++;
  }
}
