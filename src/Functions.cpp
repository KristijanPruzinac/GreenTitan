#include "Functions.h"

void Warning(String message){
  Serial.println("WARNING: " + message);
}

void Error(String message){
  Serial.println("ERROR: " + message);

  //TODO: Implement additional user feedback and error handling
  
  while (1){
    delay(100);
  }
  
}