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
#include <SPI.h> //necessário para o ADS1248
#include <math.h>

#define DEBUG 
#undef DEBUG 

/*********** definições de constantes do programa *******************/
#define SERIAL_BAUDRATE 9600
#define SERIAL_PACKET_SIZE 64 //64 bits o protocolo de comunicação com o PC
#define RESOLUCAO_ADC 12
#define AMOSTRAS_MEDIA 5 //média aritmética de 5 amostras por canal
#define ADS1248_NCANAIS 4
#define LCD_SERIAL_CADENCIA 1000
#define ADS1248_CADENCIA 500
#define ADC_RESOLUCAO 12
#define PWM_RESOLUCAO 12
#define PID_MAX 4095
#define PID_MIN 0

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

//PID constantes
#define KP  2.0
#define KI  5.0
#define KD  1.0

//tipos de sensores, usado no pacote serial
#define SENSOR_TERMISTOR 0
#define SENSOR_ADC 1
#define SENSOR_TERMOPAR 2
#define SENSOR_ADS 3

//costantes definidas por Saulo Guths
#define C_SUP 0.288
#define C_INF 0.259

//pinos utilizados (interface I2C): 20 (SDA), 21(SCL) 
LiquidCrystal_I2C lcd(0x27,20,4);

/* Declaração de variáveis e pinos usados */
const uint8_t LED_PID = 46;
const uint8_t LED_STATUS = 13;
const uint8_t ADS_SLAVE_SELECT = 47;
const uint8_t CONTROLE_POT = A2;
const uint8_t TERM_PLACA = A0;
const uint8_t TERM_AR = A1;
const uint8_t PID_PWM = 12;
const float TETO_ESCALA_DELTA_T = 30.0;

typedef struct {
    uint8_t preamble_init;
    uint8_t sensor_type;
    uint8_t sensor_number;
    double sensor_data;
    uint8_t preamble_end;
} packet;

//Variáveis auxiliares
//temporizacoes
long long lcd_serial_cadencia_ms = 0; //intervalo para printar a temperatura através da serial (1s)
long long ads1248_cadencia_ms = 0;
long long tempo_ms = 0;

//variaveis do ads1248 e dos termistores
double ads1248_ganhoCanal[4]; //vetor de configuração de ganho por canal
double ads1248_ad[4]; //4 canais com 8 entradas diferenciais
double media_tPlaca = 0, media_tAr = 0;
double temp_tPlaca = 0, temp_tAr = 0;
double q_sup=0, q_inf=0;

//pid
int pid_out = 0;
double erro = 0, deltaT_placas = 0;
double deltaT_usuario = 0, deltaT_adc=0;

void setup() {   
  
    //Serial que se comunica com o software no PC
    Serial.begin(SERIAL_BAUDRATE);

    pinMode(LED_STATUS, OUTPUT);
    digitalWrite(LED_STATUS, LOW);

    pinMode(LED_PID, OUTPUT);
    digitalWrite(LED_PID, LOW);

    analogReadResolution(ADC_RESOLUCAO); //ADC 12 bit resolution 0-4096
    analogWriteResolution(PWM_RESOLUCAO); //DAC 12 bit resolution 0-4096
    pinMode(CONTROLE_POT, INPUT);
    pinMode(TERM_PLACA, INPUT);
    pinMode(TERM_AR, INPUT);
    pinMode(PID_PWM, OUTPUT);

    ads1248_ganhoCanal[0] = 4;  //2 elev 4 = 16 (datasheet)
    ads1248_ganhoCanal[1] = 4;
    ads1248_ganhoCanal[2] = 4;
    ads1248_ganhoCanal[3] = 4;

    //SPI
    pinMode(ADS_SLAVE_SELECT, OUTPUT);
    digitalWrite(ADS_SLAVE_SELECT, HIGH);
    SPI_init();
    ads1248_clear();

    //Inicialização do display LCD
    lcd.init();
    lcd.home();
    lcd.backlight();
    
    //Mostra a tela de inicialização
    lcd_splash();

    //Monta do template do display LCD para o experimento da conveccao
    templateLCD_conveccao();

    //inicializa o display com as leituras zeradas
    escreveTensao_lcd(0, 0);
    escreveTensao_lcd(0, 1);

    escreveTemp_lcd(0, 2);
    escreveTemp_lcd(0, 3);
}

void loop() {
    tempo_ms = millis();

    //Calcula a media de leituras do ADC antes de converter em graus Celsius
    media_tPlaca = calculaMedia(TERM_PLACA, AMOSTRAS_MEDIA);
    media_tAr = calculaMedia(TERM_AR, AMOSTRAS_MEDIA);

    temp_tPlaca = steinhartAndHart(media_tPlaca, RESISTENCIA_TERMISTOR_1, RESISTENCIA_PAD_T1);
    temp_tAr = steinhartAndHart(media_tAr, RESISTENCIA_TERMISTOR_2, RESISTENCIA_PAD_T2);     
    deltaT_adc = analogRead(CONTROLE_POT);
    deltaT_usuario = ((deltaT_adc*TETO_ESCALA_DELTA_T)/4095.0);
    
    #ifdef DEBUG
    Serial.print("deltaT_usuario:");
    Serial.print(deltaT_usuario);
    Serial.println("");
    #endif
    
    deltaT_placas = temp_tPlaca - temp_tAr;
    pid_out = controle_pid(deltaT_usuario, deltaT_placas);
    analogWrite(PID_PWM, pid_out);

    if ((tempo_ms - ads1248_cadencia_ms) >= ADS1248_CADENCIA) {

        ads1248_ad[0] = ads1248_ler_amplificador(0);
        ads1248_ad[1] = ads1248_ler_amplificador(1);
        ads1248_ad[2] = ads1248_ler_amplificador(2);
        ads1248_ad[3] = ads1248_ler_amplificador(3);
        ads1248_cadencia_ms = tempo_ms;
    }

    q_sup = C_SUP * ads1248_ad[2];
    q_inf = C_INF * ads1248_ad[3];

    if ((tempo_ms - lcd_serial_cadencia_ms) >= LCD_SERIAL_CADENCIA) {
        //LCD
        //Converte em graus Celsius utilizando a função de SteinHart-Hart e escreve no LCD
        escreveTemp_lcd(temp_tPlaca, 2);
        escreveTemp_lcd(temp_tAr, 3);
        escreveTensao_lcd(q_sup, 0);
        escreveTensao_lcd(q_inf, 1);
        enviaSerial(SENSOR_TERMISTOR, 0,temp_tPlaca);
        enviaSerial(SENSOR_TERMISTOR, 1,temp_tAr);
        enviaSerial(SENSOR_ADS, 0, q_sup);
        enviaSerial(SENSOR_ADS, 1, q_inf);
        lcd_serial_cadencia_ms = tempo_ms;
    }
}

/* Splash screen do LCD ao inicializar o sistema, apresentando as informações da instituição de ensino
 *  do laboratório e do departamento.
 */
void lcd_splash() {
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
    delay(1000);
    lcd.clear();
}

//As funções do LCD desta biblioteca não retornam erro, por isso declaro como void
void templateLCD_conveccao() {
    lcd.setCursor(0, 0);
    lcd.print("q_sup:");
    delay(100); //era 100 pra tudo
    lcd.setCursor(16, 0);
    lcd.print("W/m2");
    lcd.setCursor(0, 1);
    lcd.print("q_inf:");
    delay(100);
    lcd.setCursor(16, 1);
    lcd.print("W/m2");
    delay(100);
    lcd.setCursor(0, 2);
    lcd.print("T_placa:");
    delay(100);
    lcd.setCursor(17, 2);
    lcd.print((char) 223); // degree symbol
    lcd.print("C");
    delay(100);
    lcd.setCursor(0, 3);
    lcd.print("T_ar:");
    lcd.setCursor(17, 3);
    lcd.print((char) 223); // degree symbol
    lcd.print("C");

}

void escreveTensao_lcd(float tensao, uint8_t linha_display) {
    lcd.setCursor(8, linha_display); 
    //if (tensao > 10000 || tensao < 0) tensao = 0;
    lcdPrintFloat(tensao, 1);
}

void escreveTemp_lcd(double temperatura, uint8_t linha_display) {
    lcd.setCursor(12, linha_display); 
    lcdPrintFloat(temperatura, 1);
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
    Serial.print(serialOut.sensor_data,2);
    Serial.print(serialOut.preamble_end);
    Serial.println("");
}

/* Init SPI interface*/
void SPI_init() {
    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
    SPI.setClockDivider(84);
}

/* clear ads1248 registers */
void ads1248_clear() {
    digitalWrite(ADS_SLAVE_SELECT, LOW);
    SPI.transfer(0x06);
    delay(2);
    digitalWrite(ADS_SLAVE_SELECT, HIGH);
    digitalWrite(ADS_SLAVE_SELECT, LOW);
    SPI.transfer(0x41);
    SPI.transfer(0X00);
    SPI.transfer(0xAA);  //??
    digitalWrite(ADS_SLAVE_SELECT, HIGH);
    digitalWrite(ADS_SLAVE_SELECT, LOW);
    SPI.transfer(0x42);
    SPI.transfer(0X00);
    SPI.transfer(0x30); //??
    digitalWrite(ADS_SLAVE_SELECT, HIGH);
}

double ads1248_ler_amplificador(uint8_t canal) {
    int canal_positivo_ads1248, canal_negativo_ads1248;
    bool scratch_ads1248; //aparentemente eh uma variavel sem uso
    double temp_ads1248_ad = 0;

    canal_positivo_ads1248 = canal * 2;
    canal_negativo_ads1248 = (canal * 2) + 1;
    scratch_ads1248 = ads1248_ajustar_canal_ganho(canal_positivo_ads1248, canal_negativo_ads1248, ads1248_ganhoCanal[canal]);
    digitalWrite(LED_STATUS, HIGH);
    delay(200);
    digitalWrite(LED_STATUS, LOW);
    delay(200);
    temp_ads1248_ad = (ads1248_ler(ads1248_ganhoCanal[canal]) * 1000000); //converte uV para numeros legíveis (1e6)      
    return temp_ads1248_ad;
}

bool ads1248_ajustar_canal_ganho(int MUX_SP, int MUX_SN, int PGA) {
    int BCS, DOR;
    int MUX0_word_write, SYS0_word_write, MUX0_word_read, SYS0_word_read;
    int contador_tentativas;
    bool registro_MUX0_ok, registro_SYS0_ok;
    registro_MUX0_ok = false;
    registro_SYS0_ok = false;
    contador_tentativas = 0;
    while (registro_MUX0_ok == false && contador_tentativas < 10) {
        BCS = 0;
        MUX0_word_write = (BCS << 6) | (MUX_SP << 3) | MUX_SN;
        digitalWrite(ADS_SLAVE_SELECT, LOW);
        SPI.transfer(0x40);
        SPI.transfer(0x00);
        SPI.transfer(MUX0_word_write);
        digitalWrite(ADS_SLAVE_SELECT, HIGH);
        digitalWrite(ADS_SLAVE_SELECT, LOW);
        SPI.transfer(0x20);
        SPI.transfer(0x00);
        MUX0_word_read = SPI.transfer(0xFF);
        digitalWrite(ADS_SLAVE_SELECT, HIGH);
        if (MUX0_word_write == MUX0_word_read)
            registro_MUX0_ok = true;
        contador_tentativas++;
    }
    contador_tentativas = 0;
    while (registro_SYS0_ok == false && contador_tentativas < 10) {
        DOR = 0;
        SYS0_word_write = (PGA << 4) | DOR;
        digitalWrite(ADS_SLAVE_SELECT, LOW);
        SPI.transfer(0x43);
        SPI.transfer(0X00);
        SPI.transfer(SYS0_word_write);
        digitalWrite(ADS_SLAVE_SELECT, HIGH);
        digitalWrite(ADS_SLAVE_SELECT, LOW);
        SPI.transfer(0x23);
        SPI.transfer(0x00);
        SYS0_word_read = SPI.transfer(0xFF);
        digitalWrite(ADS_SLAVE_SELECT, HIGH);
        if (SYS0_word_write == SYS0_word_read)
            registro_SYS0_ok = true;
        contador_tentativas++;
    }
    if (registro_MUX0_ok && registro_SYS0_ok) {
        return true;
    } else {
        return false;
    }
}

double ads1248_ler(int ganho) {
    int MSB, midSB, LSB;
    unsigned long int palavra;
    double tensao_ads1248[200];
    double media_tensao_ads1248, media_tensao_ads1248_corrigida;
    int contador_tensoes_ok, numero_leitura_media;
    numero_leitura_media = 100;
    media_tensao_ads1248 = 0;
    media_tensao_ads1248_corrigida = 0;
    contador_tensoes_ok = 0;
    for (int leituras = 1; leituras <= numero_leitura_media; leituras++) {
        digitalWrite(ADS_SLAVE_SELECT, LOW);
        SPI.transfer(0x12);
        MSB = SPI.transfer(0xFF);
        midSB = SPI.transfer(0xFF);
        LSB = SPI.transfer(0xFF);
        digitalWrite(ADS_SLAVE_SELECT, HIGH);
        palavra = (MSB << 16) | (midSB << 8) | LSB;
        palavra = palavra & 0x7FFFFF;
        tensao_ads1248[leituras] = palavra;
        tensao_ads1248[leituras] = tensao_ads1248[leituras] * 2.048 / 8388607;
        if ((MSB >> 7) == 1)
            tensao_ads1248[leituras] = (tensao_ads1248[leituras] - 2.048) / pow(2, ganho);
        if ((MSB >> 7) == 0)
            tensao_ads1248[leituras] = (tensao_ads1248[leituras]) / pow(2, ganho);
        media_tensao_ads1248 = media_tensao_ads1248 + tensao_ads1248[leituras];
    }
    media_tensao_ads1248 = media_tensao_ads1248 / numero_leitura_media;
    for (int leituras = 1; leituras <= numero_leitura_media; leituras++) {
        if (tensao_ads1248[leituras] >= (media_tensao_ads1248 - media_tensao_ads1248 * 0.1) && tensao_ads1248[leituras] <= (media_tensao_ads1248 + media_tensao_ads1248 * 0.1)) {
            media_tensao_ads1248_corrigida = media_tensao_ads1248_corrigida + tensao_ads1248[leituras];
            contador_tensoes_ok++;
        }
    }
    if (contador_tensoes_ok != 0) {
        media_tensao_ads1248_corrigida = media_tensao_ads1248_corrigida / contador_tensoes_ok;
        return media_tensao_ads1248_corrigida;
    } else {
        return media_tensao_ads1248;
    }
}

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

//função recebe como argumento o setpoint e a entrada atual a serem inseridos no controlador e responde em numero inteiro 0-4096 para aplicar direto no DAC de 12 bits
int controle_pid(double setpoint, double input) {
    double kp=KP, ki=KI, kd=KD; 
    static double p_term = 0, i_term = 0, d_term = 0, pid = 0, last_input=0;
    static double erro = 0, erro_old = 0;
    int pid_value = 0;

    erro = setpoint - input;
    if (erro < 1 && erro > -1) digitalWrite(LED_PID, HIGH);
    else digitalWrite(LED_PID, !digitalRead(LED_PID));
    #ifdef DEBUG
    Serial.print("erro:");
    Serial.print(erro);
    Serial.println("");
    #endif
    p_term = kp*erro;
    i_term += (ki*erro);
    if(i_term > PID_MAX) i_term=PID_MAX;
    else if(i_term < PID_MIN) i_term = PID_MIN;
    d_term = kd*(input-last_input);
    pid = p_term + i_term - d_term;
    pid = 4095 - pid;
    #ifdef DEBUG
    Serial.print("pid_tmp:");
    Serial.print(pid);
    Serial.println("");
    #endif
    if(pid > PID_MAX) pid=PID_MAX;
    if(pid < PID_MIN) pid=PID_MIN; 
    #ifdef DEBUG
    Serial.print("pid:");
    Serial.print(pid);
    Serial.println("");
    #endif
    erro_old = erro;
    last_input = input;
    pid_value = pid;
    return pid_value;
  }

/* int steinhartAndHart(int mediaLeituras, int resistenciaTermisor, int resistenciaPad);
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
