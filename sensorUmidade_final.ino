#define DISCARDED_SAMPLES 5
#define N_SAMPLES 5
#define FN_SAMPLES 5.0
#define SERIAL_CADENCIA 60000 //1 min
#define AMOSTRAGEM_CADENCIA 30000 //amostragem a cada 30s
//o sensor da gardenBot trabalha em torno de um sistema de inversao de tensao de alimentacao, cuja finalidade eh sanar o problema de corrosao ocasionado pela alimentacao 
//ininterrupta perceptivel em grande maioria dos sensores de umidade do solo
const int gardenBot_adc = 0;    //sms: soil moisture sensor, sensor de medição de umidade do solo fabricado no LMPT (http://gardenbot.org/howTo/soilMoisture/#The_local_circuit_-_simple_voltage)
const int gardenBot_voltage_direct = 6;
const int gardenBot_voltage_reverse = 7;
const int gardenBot_flipTimer = 1000;

double gardenBot_direct = 0;
double gardenBot_reverse = 0;
double gardenBot_average = 0;

const int echo10_adc = 1; //sensor de umidade do solo ECHO-10, da Decagon Devices 
const int echo10_trigger = 4; //o sensor echo10 precisa ser trigado por pelo menos 10ms a uma tensão entre 2 e 5V, sendo o valor de saída (leitura) uma variação entre 10 a 40% da tensão de trigger
double echo10_read = 0;  // variable to store the value coming from the sensor

long long serial_cadencia_ms = 0;
long long amostragem_cadencia_ms = 0;
long long tempo_ms = 0;
int i = 0;

void setup() {
  
   Serial.begin(9600);
   pinMode(echo10_trigger, OUTPUT); 
   pinMode(echo10_adc, INPUT);
   digitalWrite(echo10_trigger, LOW);
   
  pinMode(gardenBot_voltage_direct, OUTPUT);
  pinMode(gardenBot_voltage_reverse, OUTPUT);
  pinMode(gardenBot_adc, INPUT);
  
  analogRead(echo10_adc); //a primeira leitura eh descartada pois nao eh precisa
  //isto eh um problema especifico do atmega328p
}
 
void loop() {
  tempo_ms = millis();

  if ((tempo_ms - amostragem_cadencia_ms) >= AMOSTRAGEM_CADENCIA) {
      //o sensor da gardenBot trabalha com tensao de referencia em 5V, default do Arduino UNO
      analogReference(DEFAULT);
      delay(20);   
      
      //eh necessario descartar algumas amostras apos mudar a referencia do ADC devido a problemas de precisao
      for(i=0; i<DISCARDED_SAMPLES; i++) 
        analogRead(gardenBot_adc);
      
      gardenBot_reverse = 0;
      gardenBot_direct = 0;
      gardenBot_average = 0;
      
      for(i=0; i<N_SAMPLES; i++) {  
     
        setSensorPolarity(true);
        delay(gardenBot_flipTimer);
        //leitura em sentido direto da corrente
        gardenBot_direct += analogRead(gardenBot_adc);
        
        delay(gardenBot_flipTimer);  
        setSensorPolarity(false);
        delay(gardenBot_flipTimer);
        //leitura em sentido reverso
        gardenBot_reverse += 1023.0 - analogRead(gardenBot_adc);
      }
      
      gardenBot_direct = gardenBot_direct/FN_SAMPLES;
      gardenBot_reverse = gardenBot_reverse/FN_SAMPLES;
      gardenBot_average = (gardenBot_direct + gardenBot_reverse)/2.0;
    
      //como a tensão de alimentação do sensor echo10 eh 2,5V (por divisor de tensao em hdw), o valor maximo de leiura do adc eh de 1V
      //portanto, usa-se a tensao de referencia interna do Arduino UNO de 1V1 para aproveitar o maximo de resolucao possivel 
      analogReference(INTERNAL);
      delay(20);   
      
      //eh necessario descartar algumas amostras apos mudar a referencia do ADC devido a problemas de precisao
      for(i=0; i<DISCARDED_SAMPLES; i++) 
        analogRead(echo10_adc);
        
      echo10_read = 0;
    
      for(i=0; i<N_SAMPLES; i++) {
    
        digitalWrite(echo10_trigger, HIGH);
        delay(15); 
        echo10_read += analogRead(echo10_adc); 
        delay(15); 
        digitalWrite(echo10_trigger, LOW);  
    }
      echo10_read = echo10_read/FN_SAMPLES;   
      amostragem_cadencia_ms = tempo_ms;
  } 
  
  if ((tempo_ms - serial_cadencia_ms) >= SERIAL_CADENCIA) {
    
      Serial.print("gardenBot: " );    
      Serial.println(gardenBot_average); 
      gardenBot_average = gardenBot_average*5.376e-3;
      Serial.print("gardenBot (V): " );
      Serial.println(gardenBot_average);

      Serial.print("echo10: " );    
      Serial.println(echo10_read);  
      Serial.print("echo10 (V): " );
      echo10_read = ((echo10_read*1.226e-3) -  154.761e-3);  
      Serial.println(echo10_read); 
      Serial.println("");
      serial_cadencia_ms = tempo_ms;
  }
}

void setSensorPolarity(boolean flip){
  if(flip){
    digitalWrite(gardenBot_voltage_direct, HIGH);
    digitalWrite(gardenBot_voltage_reverse, LOW);
  }else{
    digitalWrite(gardenBot_voltage_direct, LOW);
    digitalWrite(gardenBot_voltage_reverse, HIGH);
  }
}

