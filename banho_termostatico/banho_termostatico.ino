/*  ********************************************************************************
 *   LMPT-UFSC: Software para Controle Banho Termostático por Relé
 Autor: Victor Augusto dos Santos
 E-Mail: viic.santos@gmail.com
 
 Funcionamento: O software faz a amostragem de dois termistores acoplados aos conversores AD do 
 Arduino Nano e utiliza uma lógica PID para ligar ou desligar duas resistências aquecedores (rabo quente)
 conectadas aos relés.

 Neste experimento são usados termistores de 10K Ohms, com uma resistência auxiliar de mesma grandeza.
 
 ******************************************************************************** */

/*********** definições de constantes do programa *******************/
#define AMOSTRAS_MEDIA 5.0 //média aritmética de 5 amostras por canal
#define PID_MAX 1023.0
#define PID_MIN 0
#define FUNDO_ESCALA_ADC 1023.0
#define VMAX_IO 5.0
#define TEMP_THRESHOLD_MINOR 1.0
#define TEMP_THRESHOLD_MAJOR 3.0
#define A_HIGH 255
#define A_LOW 0
#define SERIAL_BAUDRATE 9600
#define RELAY_CADENCIA 2000 //2s

//definição dos valores de resistência nominal para cada termistor utilizado
#define RESISTENCIA_TERMISTOR_1 10000.0
#define RESISTENCIA_TERMISTOR_2 10000.0
//definição dos valores de resistência auxiliar (pad) para cada termistor em ohms
#define RESISTENCIA_PAD_T1 10000.0
#define RESISTENCIA_PAD_T2 10000.0

//constantes calibracao para o thermistor de 10k Ohm
#define A 3.354016e-03                 
#define B 3.509850e-04                 
#define C 2.620131e-06   

/* Declaração de variáveis e pinos usados */
const uint8_t RELAY_1 = A0;
const uint8_t RELAY_2 = 13;
const uint8_t CONTROLE_POT = A3;
const uint8_t TERM_BASE1 = A6;
const uint8_t TERM_BASE2 = A2;

const uint8_t UNIDADE_1 = 10;
const uint8_t UNIDADE_2 = 11;
const uint8_t UNIDADE_4 = 12;
const uint8_t UNIDADE_6 = A4;
const uint8_t UNIDADE_7 = A5;
const uint8_t UNIDADE_9 = A1;
const uint8_t UNIDADE_10 = 2;

const uint8_t DEZENA_1 = 5;
const uint8_t DEZENA_2 = 4;
const uint8_t DEZENA_4 = 3;
const uint8_t DEZENA_6 = 9;
const uint8_t DEZENA_7 = 8;
const uint8_t DEZENA_9 = 7;
const uint8_t DEZENA_10 = 6;

int dezena = 0, unidade = 0;

double media_t1 = 0, media_t2 = 0, temp_t1 = 0, temp_t2 = 0, tempUsuario_adc = 0, tempUsuario = 0, media_termistores = 0, deltaT = 0;
const float TETO_ESCALA_DELTA_T = 99.0;

//constantes de calibração  do termistor
const double a =56.83785716;
const double b =-446.8507411;
const double c=1372.40215;  
const double d=-2053.311767;  
const double e=1525.70188;  
const double f=-449.8281345;

long long relay_cadencia_ms = 0;
long long tempo_ms = 0;

struct num7seg{
    int dezena;
    int unidade;
};

struct num7seg splitNumber(double number);
void escreve7seg(int dezena, int unidade);

struct num7seg res_split;

void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    analogReference(DEFAULT); //5V

    pinMode(RELAY_1, OUTPUT);
    digitalWrite(RELAY_1, HIGH);
    pinMode(RELAY_2, OUTPUT);
    digitalWrite(RELAY_2, HIGH);

    pinMode(CONTROLE_POT, INPUT);
    pinMode(TERM_BASE1, INPUT);
    pinMode(TERM_BASE2, INPUT);

    //displays
    pinMode(UNIDADE_1, OUTPUT);
    pinMode(UNIDADE_2, OUTPUT);
    pinMode(UNIDADE_4, OUTPUT);
    pinMode(UNIDADE_6, OUTPUT);
    pinMode(UNIDADE_7, OUTPUT);
    pinMode(UNIDADE_9, OUTPUT);
    pinMode(UNIDADE_10, OUTPUT);
    pinMode(DEZENA_1, OUTPUT);
    pinMode(DEZENA_2, OUTPUT);
    pinMode(DEZENA_4, OUTPUT);
    pinMode(DEZENA_6, OUTPUT);
    pinMode(DEZENA_7, OUTPUT);
    pinMode(DEZENA_9, OUTPUT);
    pinMode(DEZENA_10, OUTPUT);

    digitalWrite(UNIDADE_1, LOW);
    digitalWrite(UNIDADE_2, LOW);
    digitalWrite(UNIDADE_4, LOW);
    digitalWrite(UNIDADE_6, LOW);
    digitalWrite(UNIDADE_7, LOW);
    digitalWrite(UNIDADE_9, LOW);
    digitalWrite(UNIDADE_10, LOW);
    digitalWrite(DEZENA_1, LOW);
    digitalWrite(DEZENA_2, LOW);
    digitalWrite(DEZENA_4, LOW);
    digitalWrite(DEZENA_6, LOW);
    digitalWrite(DEZENA_7, LOW);
    digitalWrite(DEZENA_9, LOW);
    digitalWrite(DEZENA_10, LOW);

}

void loop() {
    tempo_ms = millis();

    //Calcula a media de leituras do ADC antes de converter em graus Celsius
    media_t1 = calculaMedia(TERM_BASE1, AMOSTRAS_MEDIA);
    media_t2 = calculaMedia(TERM_BASE2, AMOSTRAS_MEDIA);
    
    //media_t1 = (media_t1 * VMAX_IO)/FUNDO_ESCALA_ADC; //passa para V em double para usar a função de calibração
    //temp_t1 = calibrar_tensao(media_t1);
    temp_t1 = steinhartAndHart(media_t1, RESISTENCIA_TERMISTOR_1, RESISTENCIA_PAD_T1);

    //media_t2 = (media_t2 * VMAX_IO)/FUNDO_ESCALA_ADC; //passa para V em double para usar a função de calibração
    //temp_t2 = calibrar_tensao(media_t2);
    temp_t2 = steinhartAndHart(media_t2, RESISTENCIA_TERMISTOR_2, RESISTENCIA_PAD_T2);

    media_termistores = (temp_t1 + temp_t2)/2;
       
    tempUsuario_adc = analogRead(CONTROLE_POT);
    tempUsuario = ((tempUsuario_adc*TETO_ESCALA_DELTA_T)/FUNDO_ESCALA_ADC);

    res_split = splitNumber(tempUsuario);
    escreve_7seg(res_split.dezena, res_split.unidade);

    deltaT = tempUsuario - media_termistores;

   if ((tempo_ms - relay_cadencia_ms) >= RELAY_CADENCIA) {

    Serial.println("termistor_1");
    Serial.print(temp_t1);
    Serial.println("");

    Serial.println("termistor_2");
    Serial.print(temp_t2);
    Serial.println("");

    Serial.println("tempUsuario");
    Serial.print(tempUsuario);
    Serial.println("");

    Serial.println("deltaT");
    Serial.print(deltaT);
    Serial.println("");
    
    #if 1
    if (deltaT <= TEMP_THRESHOLD_MINOR) {
        digitalWrite(RELAY_1, HIGH);
        digitalWrite(RELAY_2, HIGH);
    } 
    else if (deltaT >= TEMP_THRESHOLD_MINOR && deltaT <= TEMP_THRESHOLD_MAJOR) {
        digitalWrite(RELAY_1, HIGH);
        digitalWrite(RELAY_2, LOW);   
   }
   else if (deltaT > TEMP_THRESHOLD_MAJOR){
        digitalWrite(RELAY_1, LOW);
        digitalWrite(RELAY_2, LOW); 
   }
   #endif
        relay_cadencia_ms = tempo_ms;
    }
 
}


/* int calculaMedia(int entrada, int n_amostras);   
 *  A função retorna a média, em número inteiro, de n amostragens da entrada especificada
 */
double calculaMedia(double entrada, int n_amostras) {
    int a = 0;
    int media_leituras = 0;

    for (a = 0; a < n_amostras; a++) {
        media_leituras += analogRead(entrada);
    }
    media_leituras /= n_amostras;
    return media_leituras;
}


//função escrita pelo Saulo com as constantes de calibração do termistor de 10k (levantada por aproximacao no Excel)
//a função recebe um valor double de tensão e retorna a respectiva temperatura em Celsius referente a esta tensão
double calibrar_tensao(double tensao_para_calibrar) {
  double retorno_calibracao;
  retorno_calibracao = a*pow(tensao_para_calibrar, 5) +
                       b*pow(tensao_para_calibrar, 4) +
                       c*pow(tensao_para_calibrar, 3) +
                       d*pow(tensao_para_calibrar, 2) +
                       e*pow(tensao_para_calibrar, 1) +
                       f*pow(tensao_para_calibrar, 0);
  return retorno_calibracao; 
}

void escreve_7seg(int dezena, int unidade) {

      switch(unidade) {
      case 0:
          digitalWrite(UNIDADE_1, HIGH);
    digitalWrite(UNIDADE_2, HIGH);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_HIGH);
    analogWrite(UNIDADE_7, A_HIGH);
    analogWrite(UNIDADE_9, A_HIGH);
    digitalWrite(UNIDADE_10, LOW);
        break;

      case 1:
          digitalWrite(UNIDADE_1, LOW);
    digitalWrite(UNIDADE_2, LOW);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_HIGH);
    analogWrite(UNIDADE_7, A_LOW);
    analogWrite(UNIDADE_9, A_LOW);
    digitalWrite(UNIDADE_10, LOW);
        break;

      case 2:
          digitalWrite(UNIDADE_1, HIGH);
    digitalWrite(UNIDADE_2, HIGH);
    digitalWrite(UNIDADE_4, LOW);
    analogWrite(UNIDADE_6, A_HIGH);
    analogWrite(UNIDADE_7, A_HIGH);
    analogWrite(UNIDADE_9, A_LOW);
    digitalWrite(UNIDADE_10, HIGH);
        break;
            
      case 3:
          digitalWrite(UNIDADE_1, LOW);
    digitalWrite(UNIDADE_2, HIGH);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_HIGH);
    analogWrite(UNIDADE_7, A_HIGH);
    analogWrite(UNIDADE_9, A_LOW);
    digitalWrite(UNIDADE_10, HIGH);
        break;
        
            case 4:
                digitalWrite(UNIDADE_1, LOW);
    digitalWrite(UNIDADE_2, LOW);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_HIGH);
    analogWrite(UNIDADE_7, A_LOW);
    analogWrite(UNIDADE_9, A_HIGH);
    digitalWrite(UNIDADE_10, HIGH);
      break;
      
            case 5:
                digitalWrite(UNIDADE_1, LOW);
    digitalWrite(UNIDADE_2, HIGH);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_LOW);
    analogWrite(UNIDADE_7, A_HIGH);
    analogWrite(UNIDADE_9, A_HIGH);
    digitalWrite(UNIDADE_10, HIGH);
      break;
      
            case 6:
                digitalWrite(UNIDADE_1, HIGH);
    digitalWrite(UNIDADE_2, HIGH);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_LOW);
    analogWrite(UNIDADE_7, A_HIGH);
    analogWrite(UNIDADE_9, A_HIGH);
    digitalWrite(UNIDADE_10, HIGH);
      break;
            case 7:
     digitalWrite(UNIDADE_1, LOW);
    digitalWrite(UNIDADE_2, LOW);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_HIGH);
    analogWrite(UNIDADE_7, A_HIGH);
    analogWrite(UNIDADE_9, A_LOW);
    digitalWrite(UNIDADE_10, HIGH);
      break;
      
            case 8:
                digitalWrite(UNIDADE_1, HIGH);
    digitalWrite(UNIDADE_2, HIGH);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_HIGH);
    analogWrite(UNIDADE_7, A_HIGH);
    analogWrite(UNIDADE_9, A_HIGH);
    digitalWrite(UNIDADE_10, HIGH);
      break;
      
            case 9:
                digitalWrite(UNIDADE_1, LOW);
    digitalWrite(UNIDADE_2, HIGH);
    digitalWrite(UNIDADE_4, HIGH);
    analogWrite(UNIDADE_6, A_HIGH);
    analogWrite(UNIDADE_7, A_HIGH);
    analogWrite(UNIDADE_9, A_HIGH);
    digitalWrite(UNIDADE_10, HIGH);
      break;

      default: break;
      
    }

       switch(dezena) {
      case 0:
          digitalWrite(DEZENA_1, HIGH);
    digitalWrite(DEZENA_2, HIGH);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, HIGH);
    digitalWrite(DEZENA_7, HIGH);
    digitalWrite(DEZENA_9, HIGH);
    digitalWrite(DEZENA_10, LOW);
        break;

      case 1:
          digitalWrite(DEZENA_1, LOW);
    digitalWrite(DEZENA_2, LOW);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, HIGH);
    digitalWrite(DEZENA_7, LOW);
    digitalWrite(DEZENA_9, LOW);
    digitalWrite(DEZENA_10, LOW);
        break;

      case 2:
          digitalWrite(DEZENA_1, HIGH);
    digitalWrite(DEZENA_2, HIGH);
    digitalWrite(DEZENA_4, LOW);
    digitalWrite(DEZENA_6, HIGH);
    digitalWrite(DEZENA_7, HIGH);
    digitalWrite(DEZENA_9, LOW);
    digitalWrite(DEZENA_10, HIGH);
        break;
            
      case 3:
          digitalWrite(DEZENA_1, LOW);
    digitalWrite(DEZENA_2, HIGH);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, HIGH);
    digitalWrite(DEZENA_7, HIGH);
    digitalWrite(DEZENA_9, LOW);
    digitalWrite(DEZENA_10, HIGH);
        break;
        
            case 4:
                digitalWrite(DEZENA_1, LOW);
    digitalWrite(DEZENA_2, LOW);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, HIGH);
    digitalWrite(DEZENA_7, LOW);
    digitalWrite(DEZENA_9, HIGH);
    digitalWrite(DEZENA_10, HIGH);
      break;
      
            case 5:
                digitalWrite(DEZENA_1, LOW);
    digitalWrite(DEZENA_2, HIGH);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, LOW);
    digitalWrite(DEZENA_7, HIGH);
    digitalWrite(DEZENA_9, HIGH);
    digitalWrite(DEZENA_10, HIGH);
      break;
      
            case 6:
                digitalWrite(DEZENA_1, HIGH);
    digitalWrite(DEZENA_2, HIGH);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, LOW);
    digitalWrite(DEZENA_7, HIGH);
    digitalWrite(DEZENA_9, HIGH);
    digitalWrite(DEZENA_10, HIGH);
      break;
            case 7:
     digitalWrite(DEZENA_1, LOW);
    digitalWrite(DEZENA_2, LOW);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, HIGH);
    digitalWrite(DEZENA_7, HIGH);
    digitalWrite(DEZENA_9, LOW);
    digitalWrite(DEZENA_10, HIGH);
      break;
      
            case 8:
                digitalWrite(DEZENA_1, HIGH);
    digitalWrite(DEZENA_2, HIGH);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, HIGH);
    digitalWrite(DEZENA_7, HIGH);
    digitalWrite(DEZENA_9, HIGH);
    digitalWrite(DEZENA_10, HIGH);
      break;
      
            case 9:
                digitalWrite(DEZENA_1, LOW);
    digitalWrite(DEZENA_2, HIGH);
    digitalWrite(DEZENA_4, HIGH);
    digitalWrite(DEZENA_6, HIGH);
    digitalWrite(DEZENA_7, HIGH);
    digitalWrite(DEZENA_9, HIGH);
    digitalWrite(DEZENA_10, HIGH);
      break;

      default: break;
      
    }
}

struct num7seg splitNumber(double number) {
    struct num7seg result;
    int valor_int = 0;
    int uni = 0, dez = 0;
  
    if (number > 99) valor_int = number/100;
    else valor_int = (int) number;

    dez = valor_int/10;
    uni = valor_int % 10;
    
    result.dezena = dez;
    result.unidade = uni;
    return result;
}

/* double steinhartAndHart(double mediaLeituras, double resistenciaTermisor, double resistenciaPad);
 * A função retorna o valor de temperatura em graus Celsius
 * sendo necessário entrar com o valor de leitura do ADC (ou média das leituras) 
 * no qual o termistor está acoplado, o valor nominal de sua resistência em Ohms
 * e o valor de resistência utilizado no padding (resistência auxiliar)
 */

double steinhartAndHart(double mediaLeituras, double resistenciaTermisor, double resistenciaPad) {
    //calculo da temperatura é feito seguindo a função de SteinHart-Hart para termistores
    double resistencia = resistenciaPad * ((1023.0 / mediaLeituras) - 1.0);
    double LnRes = log(resistencia / resistenciaTermisor);
    double temperatura = 1.0 / (A + (B * LnRes) + (C * pow(LnRes, 3.0)));
    temperatura = temperatura - 273.15; //converte de graus Kelvin para graus Celsius
    return temperatura;
}

