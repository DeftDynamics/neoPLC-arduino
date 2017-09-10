   //  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ BLE Communication/Menu ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void bleManager(void)
{
  // check to see if there is anything in the buffer
  if (BLE.available())                  
  { 
    all_parsed = false;
  }
  // loop until the buffer is empty
  while (all_parsed==false)  
  {
    // notes:
    // > 64 characters per transmission (BLE limit to send in one 'line')
    // > EOL is not needed, just a completed transmission LAST CELL MUST END WITH COMMA
      while (BLE.available())
      {
        all_parsed = false;

        // create an empty char array buffer
        char buffer[21];
        for (int i = 0; i<21; i++)  {buffer[i] = 0;}
        // read characters into the buffer until we receive a new line
        int i = 0;
        while (BLE.available())
        {
          char c = BLE.read(); // read a byte from the characteristic
            if (c!='\n')       // look for a newline
            {
              buffer[i] = c;   // if it's not a newline, add to char buffer
              i++;
            }
            else if (i>0)      // break if the newline is NOT the first character (since this will happen with every line read)
            {
              break;
            }
        }
        // convert to a String so we have nice tools to manipulate the line
        String BUFFER = buffer; 
        // find the start of a sentence
        int start_point = BUFFER.indexOf("$");
        // clear characters up to (not including) the '$'
        BUFFER.remove(0,start_point);
        
        // if a sentence is found (starts with '$') then parse, otherwise skip
        while ((start_point>=0)&(all_parsed == 0)&(BUFFER.indexOf(",")!=-1)) 
        {
          // print the name of the sentence, save as  String 'ID'
          int end_of_cell = BUFFER.indexOf(",");
          String ID = BUFFER.substring(0,end_of_cell);
          // print the name of the message then cut it off the String
          BUFFER.remove(0,end_of_cell+1);
          // see if there is another sentence (another "$" token) to know whether to repeat the loop
          if (BUFFER.indexOf("$")==-1) {all_parsed = true;}
          // if there is a second sentence, it will not have spurious characters, go ahead and set start_point=0
          start_point = 0;
          // report all included cells (last cell MUST end with a comma to be parsed)
          end_of_cell = BUFFER.indexOf(",");
          int cell_num = 0;
          // create an array to hold all the tokens
          String Tokens[11];
          Tokens[0] = ID;
          // pull out data from (as many as) 10 cells
          while (end_of_cell>=0 && cell_num<=9 && BUFFER.substring(0,end_of_cell).indexOf("$")==-1) 
          {
              String cell_val_string = BUFFER.substring(0,end_of_cell);
              Tokens[cell_num+1] = cell_val_string;
              BUFFER.remove(0,end_of_cell+1);
              end_of_cell = BUFFER.indexOf(",");
              cell_num++;
          }
          
//         ---------------- CHECK FOR KNOWN MESSAGES AND PARSE ------------------

//  for most users, this is the only part of the bleManager to customize.
//  data is passed here as an array of float 'tokens' from the incoming message
//  For example: "$LED,0,\r\n" is a valid message, which contains the state (0 or 1) of the LED
//  The array 'Tokens' contains all the incoming tokens converted to Floats. 'ID' contains the header
          if (Tokens[0].equalsIgnoreCase("$LED"))
          {
               led_state = pullBool(Tokens[1]);                   // LED state is received as the first token
               Serial.printf("Received: $LED: %d\n",led_state); // print a note that we received this token
               BLE.printf("$LED,%d,\r\n",led_state);         // send the phone a message, confirming the new information
          }
          else
          {
            Serial.println("invalid message ID");
          }


//         ----------------------- END OF BLE PARSING ---------------------------
          
          BLE.flush(); // flush any message
        }
      }
  }
return;
}

float pullFloat(String inString){
  return inString.toFloat();
}

int pullInt(String inString){
  return inString.toInt();
}

bool pullBool(String inString){
  bool val = inString.toInt();
  return val;
}

