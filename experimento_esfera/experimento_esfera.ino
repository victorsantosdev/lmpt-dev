/*  ********************************************************************************
 *   LMPT-UFSC: Software para Experimento de Análise de Transitório com Esferas
    Autor: Victor Augusto dos Santos
    E-Mail: viic.santos@gmail.com
    
    Funcionamento: O software faz a amostragem de três termistores acoplados aos conversores AD do 
    Arduino DUE (A0, A1 e A2) a cada 500ms. Estes termistores medem as grandezas Tnegro, Tbrilho e Tar, respectivamente.
    A cada 500ms o display LCD é atualizado, e em seguida os dados são enviados para o software de leitura no PC, através da interface serial (UART) do Arduino Due.

    Neste experimento são usados termistores de 10K Ohms, com uma resistência auxiliar de mesma grandeza.
 
    LiquidCrystal_I2C lib: https://github.com/marcoschwartz/LiquidCrystal_I2C
 ******************************************************************************** */

#include <Wire.h> //necessária para a interface i2c do display
#include <LiquidCrystal_I2C.h>
#include <math.h>

/*********** definições de constantes do programa *******************/
#define SERIAL_BAUDRATE 9600
#define SERIAL_PACKET_SIZE 64 //64 bits o protocolo de comunicação com o PC
#define RESOLUCAO_ADC 12
#define AMOSTRAS_MEDIA 5.0 //média aritmética de 5 amostras por canal
#define SERIAL_CADENCIA 1000


//definição dos valores de resistência nominal para cada termistor utilizado
#define RESISTENCIA_TERMISTOR_1 10000.0
#define RESISTENCIA_TERMISTOR_2 10000.0
#define RESISTENCIA_TERMISTOR_3 10000.0
//definição dos valores de resistência auxiliar (pad) para cada termistor em ohms
#define RESISTENCIA_PAD_T1 10000.0
#define RESISTENCIA_PAD_T2 10000.0
#define RESISTENCIA_PAD_T3 10000.0
//definição dos termistores
#define TERMISTOR_TNEGRO A0
#define TERMISTOR_TBRILHO A1
#define TERMISTOR_TAR A2
//tipos de sensores, usado no pacote serial
#define SENSOR_TERMISTOR 0
#define SENSOR_ADC 1
#define SENSOR_TERMOPAR 2
#define SENSOR_ADS 3

//constantes calibracao para o thermistor de 10k Ohm
#define A 3.354016e-03                 
#define B 3.509850e-04                 
#define C 2.620131e-06            

/* Declaração de variáveis e pinos usados */
//LCD 
//pinos utilizados (interface I2C): 20 (SDA), 21(SCL) 
LiquidCrystal_I2C lcd(0x3F,20,4); 

//Variáveis auxiliares
double media_tNegro = 0, media_tBrilho = 0, media_tAr = 0;
double temp_tNegro = 0, temp_tBrilho = 0, temp_tAr = 0;
//variáveis utilizadas no tick de print da serial
long long tempo_ms = 0;
long long serial_cadencia_ms = 0; //intervalo para printar a temperatura através da serial (1s)

typedef struct {
    uint8_t preamble_init;
    uint8_t sensor_type;
    uint8_t sensor_number;
    double sensor_data;
    uint8_t preamble_end;
  } packet;

void setup() {
  //Configuração dos pinos do ADC
  pinMode(TERMISTOR_TNEGRO, INPUT);
  pinMode(TERMISTOR_TBRILHO, INPUT);
  pinMode(TERMISTOR_TAR, INPUT);
  analogReadResolution(RESOLUCAO_ADC); //ADC resolução 12bits 0-4096

  //Serial que se comunica com o software no PC
  Serial.begin(SERIAL_BAUDRATE);

  //Inicialização do display LCD
  lcd.init();
  lcd.home();
  lcd.backlight();

  //Mostra a tela de inicialização
  lcd_splash();
  
  //Monta do template do display LCD para o experimento das esferas
  templateLCD_esfera();
  
  //inicializa as temperaturas com 0
  escreveTemp_lcd(0,1);
  escreveTemp_lcd(0,2);
  escreveTemp_lcd(0,3);
}

void loop() {

  tempo_ms = millis();
  
  //Calcula a media de leituras do ADC antes de converter em graus Celsius
  media_tNegro = calculaMedia(TERMISTOR_TNEGRO, AMOSTRAS_MEDIA);
  media_tBrilho = calculaMedia(TERMISTOR_TBRILHO, AMOSTRAS_MEDIA);
  media_tAr = calculaMedia(TERMISTOR_TAR, AMOSTRAS_MEDIA);

  temp_tNegro = steinhartAndHart(media_tNegro, RESISTENCIA_TERMISTOR_1, RESISTENCIA_PAD_T1);
  temp_tBrilho = steinhartAndHart(media_tBrilho, RESISTENCIA_TERMISTOR_2, RESISTENCIA_PAD_T2);
  temp_tAr = steinhartAndHart(media_tAr, RESISTENCIA_TERMISTOR_3, RESISTENCIA_PAD_T3);
  
  //Converte em graus Celsius utilizando a função de SteinHart-Hart e escreve no LCD
  escreveTemp_lcd(temp_tNegro,1);
  escreveTemp_lcd(temp_tBrilho,2);
  escreveTemp_lcd(temp_tAr,3);


  if ((tempo_ms - serial_cadencia_ms) >= SERIAL_CADENCIA) {
    enviaSerial(SENSOR_TERMISTOR, 0,temp_tNegro);
    enviaSerial(SENSOR_TERMISTOR, 1,temp_tBrilho);
    enviaSerial(SENSOR_TERMISTOR, 2,temp_tAr);
    serial_cadencia_ms = tempo_ms;
  }
}

/* Splash screen do LCD ao inicializar o sistema, apresentando as informações da instituição de ensino
 *  do laboratório e do departamento.
 */
void lcd_splash(){
  lcd.setCursor(8, 0);  
  lcd.print("UFSC");
  delay(100);
  lcd.setCursor(0, 1);  
  lcd.print("Dept. Eng. Mecanica");
  delay(100);
  lcd.setCursor(0, 2);  
  lcd.print("Laboratorio LMPT");
  delay(100);
  lcd.setCursor(0, 3);  
  lcd.print("Release 1.0");
  delay(5000);
  lcd.clear();
  }

//As funções do LCD desta biblioteca não retornam erro, por isso declaro como void
void templateLCD_esfera(){
  lcd.setCursor(1, 0);  
  lcd.print("Esfera Transiente");
  delay(100);
  lcd.setCursor(0, 1);  
  lcd.print("T_negro:");
  delay(100);
  lcd.setCursor(16, 1);  
  lcd.print((char)223); // degree symbol
  lcd.print("C");
  delay(100);
  lcd.setCursor(0, 2);  
  lcd.print("T_brilho:");
  delay(100);
  lcd.setCursor(16, 2);  
  lcd.print((char)223); // degree symbol
  lcd.print("C");
  delay(100);
  lcd.setCursor(0, 3);  
  lcd.print("T_ar:");
  lcd.setCursor(16, 3);
  lcd.print((char)223); // degree symbol
  lcd.print("C"); 
  }


void escreveTemp_lcd(double temperatura, uint8_t linha_display) {
    lcd.setCursor(11, linha_display); 
    lcdPrintFloat(temperatura, 1);
}


  /* utilizada na representação de números inteiros (não ponto flutuantes) */
  void escreveTemperatura_lcd(uint8_t temperatura, uint8_t linha_display) {
    if (temperatura == 0) { //condição de inicialização
      lcd.setCursor(13, linha_display);
      lcd.write('0');
      lcd.setCursor(14, linha_display);
      lcd.write('0');
      delay(100);  
    }
    else if (temperatura >= 0 && temperatura < 10) { //numero entre 0 e 10
      lcd.setCursor(13, linha_display);
      lcd.write('0');
      lcd.setCursor(14, linha_display);
      lcd.print(temperatura);
      delay(100);
    }
    else { //numeros maiores que 10
      lcd.setCursor(13, linha_display);  // column , row  column 0 , row 1 i.e. bottom line of LCD
      lcd.print(temperatura);
      delay(100);
    }
  }
  
/* double steinhartAndHart(double mediaLeituras, double resistenciaTermisor, double resistenciaPad);
 * A função retorna o valor de temperatura em graus Celsius
 * sendo necessário entrar com o valor de leitura do ADC (ou média das leituras) 
 * no qual o termistor está acoplado, o valor nominal de sua resistência em Ohms
 * e o valor de resistência utilizado no padding (resistência auxiliar)
 */

double steinhartAndHart(double mediaLeituras, double resistenciaTermisor, double resistenciaPad) {
    //calculo da temperatura é feito seguindo a função de SteinHart-Hart para termistores
    double resistencia = resistenciaPad * ((4095.0 / mediaLeituras) - 1.0);
    double LnRes = log(resistencia / resistenciaTermisor);
    double temperatura = 1.0 / (A + (B * LnRes) + (C * pow(LnRes, 3.0)));
    temperatura = temperatura - 273.15; //converte de graus Kelvin para graus Celsius
    return temperatura;
}

  /* int calculaMedia(int termistor, int n_amostras);   
 *  A função retorna a média, em número inteiro, de n amostragens da entrada especificada
   */
  double calculaMedia(double entrada, double n_amostras) {
    int i = 0;
    double media_leituras = 0;
    
    for (i=0; i < n_amostras; i++) {      
      media_leituras += analogRead(entrada);  
    }
    media_leituras /= n_amostras;
    return media_leituras;  
  }


/** lcdPrintFloat recebe como parâmetros o valor em ponto flutuante a ser printado e a precisão **/
void lcdPrintFloat(float val, byte precision) {
    // prints val on a ver 0012 text lcd with number of decimal places determine by precision
    // precision is a number from 0 to 6 indicating the desired decimial places
    // example: lcdPrintFloat( 3.1415, 2); // prints 3.14 (two decimal places)

    if (val < 0.0) {
        lcd.print('-');
        val = -val;
    }

    lcd.print((long) val);  //prints the integral part
    if (precision > 0) {
        lcd.print("."); // print the decimal point
        unsigned long frac;
        unsigned long mult = 1;
        byte padding = precision - 1;
        while (precision--)
            mult *= 10;

        if(val >= 0)
        frac = (val - int(val)) * mult;
        else
        frac = (int(val)- val ) * mult;
        unsigned long frac1 = frac;
        while (frac1 /= 10)
            padding--;
        while (padding--)
            lcd.print("0");
        lcd.print(frac, DEC);
        delay(100);
    }
}


  /* Envia o pacote com os valores de temperatura de cada sensor pela serial
  * Informar o tipo, o número e a leitura do sensor 
  *
  */
  void enviaSerial(uint8_t sensorType, uint8_t sensorNumber, double sensorData) {
      packet serialOut;
      serialOut.preamble_init = 0xAA;
      serialOut.sensor_type = sensorType;
      serialOut.sensor_number = sensorNumber;
      serialOut.sensor_data = sensorData;
      serialOut.preamble_end = 0xFF;

      Serial.print(serialOut.preamble_init);
      Serial.print(serialOut.sensor_type);
      Serial.print(serialOut.sensor_number);
      Serial.print(serialOut.sensor_data);
      Serial.print(serialOut.preamble_end);
      Serial.println("");
    }
