/**************************************
* 2-Arduino mode change               *
*                                     *
* 1:manual mode                       *
*   if Twin_Mainpin -> high           *
* 2:stabilize mode                    *
*   if Twin_Subpin  -> high           *
* 3:guided(loiter) mode               *
***************************************/

/*******************************************************
 caution!                                            
   テグスを溶断する場合は、溶断の間はmanualである必要があります。
   要するに時間的なマージンが必要。
   忘れやすいので注意。
 *******************************************************/

#define MCpin 9   //to Pixhawk(Pixracer) RC
#define Twin_Mainpin 5    //to the other Arduino D5
#define Twin_Subpin 6   //to the other Arduino D6

  int ch[8];    //PPM buffer
  int phase = 0;    //phase 0:manual 1:stabilize 2:guided(loiter)
  int manual[8] = {0,0,0,0,0,0,0,165};    //manual mode (launch ~ fall-start)
  int stabilize[8] = {0,0,0,0,0,0,0,425};   //stabilize mode (fall-start ~ nose-up)
  int guided[8] = {400,400,0,0,425,0,0,0};    //guided(loiter）mode (nose-up ~ land)

void setup(){
  Serial.begin(9600); 
  pinMode(MCpin,OUTPUT);
  pinMode(Twin_Mainpin,INPUT); //INPUT_PULLUP も存在する
  pinMode(Twin_Subpin,INPUT);
}

void loop(){
  switch(phase){
    
    case 0:   //manual mode (launch ~ fall-start)
    
      for(int i=0;i<8;i++){
        ch[i]= manual[i];   //buffer -> manual
      }
      
      /*EEPROM*/
      /*SerialPrint*/
      
      while(digitalRead(Twin_Mainpin) == LOW){   //もう一つのArduinoからの出力で抜ける
        ModeIn(ch);   //manualのPPMを送り続ける
      }
      
      /*SerialPrint*/
      phase++;
      break;

    case 1:    //stabilize mode (fall-start ~ nose-up)
    
      for(int i=0;i<8;i++){
        ch[i] = stabilize[i];   //buffer -> stabilize
      }
      
      /*EEPROM*/
      /*SerialPrint*/
      
      while(digitalRead(Twin_Subpin) == LOW){   //もう一つのArduinoからの出力で抜ける
        ModeIn(ch);   //stabilizeのPPMを送り続ける
      }
      
      /*SerialPrint*/
      phase++;
      break;

    case 2:   //guided(loiter）mode (nose-up ~ land)

      for(int i=0;i<8;i++){
          ch[i] = guided[i];   //buffer -> guided(loiter)
      }

       /*EEPROM*/
      /*SerialPrint*/
      
      while(1){   //最後まで（変更の余地がある）
        ModeIn(ch);   //guided(loiter)のPPMを送り続ける
      }

  }


 
}

/************
 * Function *
 ************/
void Pulth(int MSvalue){  //MSvalue:0~1000
  digitalWrite(MCpin,HIGH);
  delayMicroseconds(250);
  digitalWrite(MCpin,LOW);
  delayMicroseconds(750+MSvalue); 
}

void ModeIn(int MS[8]){
  int WaitZeroSum=0; 
  for(int i=0;i<8;i++){
    WaitZeroSum += MS[i]+1000;
  }  
  for(int i=0;i<8;i++){
    Pulth(ch[i]);
  }
  Pulth(20000-WaitZeroSum);
}
