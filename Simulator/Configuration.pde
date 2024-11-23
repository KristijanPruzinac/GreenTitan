//ALGORITHM SETTINGS
int MOWER_OVERLAP = 15;
int MAX_DEVIATION = 50;

void SaveConfiguration(){}
void LoadConfiguration(){

  //TODO: READ CONFIGURATION FILE (LittleFS)
  String ConfigurationData;
  
  //TODO: REMOVE SIMULATOR DATA
  ConfigurationData = new String(loadBytes("config.backup.txt"));
  
  //Parse data
  String[] lines = splitStringByCharacters(ConfigurationData, "\n");
  
  int lineIndex = 0;
  while (lineIndex < lines.length){
    //OUTLINE START
    if (lines[lineIndex].charAt(0) == 'O'){
      outlines.add(new ArrayList<ArrayList<Long>>());
    }
    else {
      String[] dataXY = splitStringByCharacters(lines[lineIndex], " ");
      
      ArrayList<ArrayList<Long>> outline = outlines.get(outlines.size() - 1);
      
      outline.add(new ArrayList<Long>());
      outline.get(outline.size() - 1).add(Long.parseLong((dataXY[0].trim()))); //WARNING: USE .toInt() in arduino
      outline.get(outline.size() - 1).add(Long.parseLong((dataXY[1].trim())));
      
      if (lineIndex == 0){
        baseLon = outline.get(outline.size() - 1).get(0);
        baseLat = outline.get(outline.size() - 1).get(1);
        
        prevPointLon = outline.get(outline.size() - 1).get(0);
        prevPointLat = outline.get(outline.size() - 1).get(1);
        
        mowerLon = outline.get(outline.size() - 1).get(0);
        mowerLat = outline.get(outline.size() - 1).get(1);
        
        targetPointLon = outline.get(outline.size() - 1).get(0);
        targetPointLat = outline.get(outline.size() - 1).get(1);
      }
    }
    
    lineIndex++;
  }
 
}
