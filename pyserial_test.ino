void setup()
{
  Serial.begin(57600); // set line end to Newline at
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(3,OUTPUT);
  digitalWrite(6,0); //left direction
  digitalWrite(7,0); //right direction
  analogWrite(3,0);
  analogWrite(5,0);
      
  Serial.println("READY");                    // bottom of serial monitor
  
}

float vals[4] = {0,0,0,0};

void set_vals(const String& string_data){
  char * pch;
  char str[20];
  strcpy(str , string_data.c_str());
  pch = strtok (str,",");
  int i = 0;
  while (pch != NULL)
  {
    vals[i] = atoi(pch);
    pch = strtok (NULL, ",");    
    i++;
  }  
}

void loop()
{
      if (Serial.available()>0){
      String data = Serial.readString(); 
      Serial.println(data);
      set_vals(data);
      digitalWrite(6,vals[2]); //left direction
      digitalWrite(7,vals[0]); //right direction
      analogWrite(3,vals[1]);
      analogWrite(5,vals[3]);
      
      }
      delay(2);
      
}
