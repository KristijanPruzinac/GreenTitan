String[] splitStringByCharacters(String input, String separators) {
  ArrayList<String> result = new ArrayList<String>();
  int startIndex = 0;

  for (int i = 0; i < input.length(); i++) {
    char currentChar = input.charAt(i);
    if (separators.indexOf(currentChar) != -1) {
      result.add(input.substring(startIndex, i));
      startIndex = i + 1;
    }
  }

  result.add(input.substring(startIndex));
  
  return result.toArray(new String[result.size()]);
}
