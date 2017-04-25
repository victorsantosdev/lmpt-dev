/*  ********************************************************************************
 *   LMPT-UFSC: Confortímetro, software utilizado para calcular o conforto térmico PMV e PPD
 Autores: Victor Augusto dos Santos, Beatriz Kloppel de Oliveira
 ******************************************************************************** */

#include <Wire.h> //necessária para a interface i2c do display
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <DS3231.h> // clock mais preciso
#include <Time.h>

/*********** definições de constantes do programa *******************/
#define AMOSTRAS_MEDIA 10.0 //quantidade de amostras do sinal analógico
//se eu fizer 10 amostras, somo elas e divido por 10
//serve para matar ruído sem precisar usar filtro analógico

//#define INTERVALO_GRAVACAO 1000 //1000ms = 1s
#define UM_MINUTO 1000//60000
#define DEZ_MINUTOS 5000//600000
#define UMA_HORA 10000//3600000
//definição dos pinos usados
#define CHAVE_PENDRIVE 3
#define CHAVE_GRAVACAO_1MIN 4
#define CHAVE_GRAVACAO_10MIN 5
#define CHAVE_GRAVACAO_60MIN 6
#define BTN_UP_METAB 8
#define BTN_DOWN_METAB 9
#define BTN_UP_IVEST 10
#define BTN_DOWN_IVEST 11
#define SENSOR_TAR A0
#define SENSOR_TGLOBO A1
#define SENSOR_VEL_AR A2
#define SENSOR_UR A3
#define PENDRIVE_SS 7
#define LED_STATUS 24 //led para o usuário ver quando o pendrive está gravando ou não
//eu que uso, não é necessário

/* Declaração de variáveis e pinos usados */
//pinos utilizados (interface I2C): 20 (SDA), 21(SCL)
LiquidCrystal_I2C lcd_primeiro(0x27,20,4);//declara que existe um lcd com endereço 0x27, e que ele tem 16 colunas e 2 linhas
LiquidCrystal_I2C lcd_segundo(0x3F,20,4);//declara que existe um lcd com endereço 0x26, e que ele tem 16 colunas e 2 linhas

//o rtc também usa o barramento I2C
DS3231 rtc( SDA, SCL);
//a estrutura Time t é usada para mostrar a hora e a data no formato
//que o Windows utiliza, e que a princípio o Excel aceita
Time t;

//Variáveis auxiliares
float tAr_filtrado = 0, tGlobo_filtrado = 0, vAr_filtrado = 0, UR_filtrado = 0;
float media_tAr = 0, media_tGlobo = 0, media_vAr = 0, media_UR = 0;
float tAr = 0, tGlobo = 0, vAr = 0, UR = 0;
int contagem_amostras = 0;
float PMV = 0, PPD = 0, metabolismo = 50, indice_de_vestimenta = 0.1;

int contador_conectado = 0;
//variáveis utilizadas no intervalo de print da serial e no date/time
long long tempo_ms = 0;
long long intervalo_gravacao_ms = 0; //intervalo para printar a temperatura através da serial (1s)
int hora, minuto, segundo, dia, mes, ano, dia_do_ano;

//variaveis auxiliares para escrever os dados no pendrive
char dados_pendrive[500];
char string_temp[500];
char nome_arquivo[50];
char arquivo_write[100];
char arquivo_append[100];
char tmp[20];

//variaveis auxiliares usadas na rotina do pendrive
int ledState = LOW;
int pendriveState = LOW;
int pendrive_primeira_vez = 0;
long long intervalo_gravacao = 60000; //valor default: 1 minuto

//máquina de estados que eu criei para a gravação no pendrive funcionar direito
enum PD_ESTADOS {
  desconectado, conectado, append, fechado
};

PD_ESTADOS pd_estadoAtual = desconectado;
PD_ESTADOS pd_UltimoEstado = desconectado;

//declaração dos prototipos das funções utilizadas
void cabecalho_LCD_confortimetro();
void Calcula_PMV(float Tar, float TGlobo, float VelocAr, float UR, float metab, float indVest, float * PMV, float * PPD);
void Read_Clock();
void cabecalho_LCD_confortimetro();
void atualiza_LCD_1(float tempAr, float tempGlobo, float VelAr, uint8_t UmRel);
void str_replace(char *s, char chr, char repl_chr);
void flash_data(char *pstring);
void str_replace(char *s, char chr, char repl_chr);
void escreve_pendrive(char * data, char* tempo, float tempAr, float tempGlobo, float VelAr, uint8_t UmRel, uint8_t metabolismo, float indice_de_vestimenta, float PMV, uint8_t PPD);
void cabecalho_pendrive();
void lcdPrintFloat(float val, byte precision, uint8_t numero_display);
float calculaMedia(float entrada, float n_amostras);
int debounce(int pin);
float calibrar_termistores(float temperatura_em_volts);
float calibrar_umidade(float umidade_em_volts);

void setup() {

  //Configuração dos pinos do ADC
  pinMode(SENSOR_TAR, INPUT);
  pinMode(SENSOR_TGLOBO, INPUT);
  pinMode(SENSOR_VEL_AR, INPUT);
  pinMode(SENSOR_UR, INPUT);

  //configuração dos botões e chave que muda o intervalo de gravação, ativos em baixo
  pinMode(CHAVE_GRAVACAO_1MIN, INPUT);
  pinMode(CHAVE_GRAVACAO_10MIN, INPUT);
  pinMode(CHAVE_GRAVACAO_60MIN, INPUT);
  pinMode(BTN_UP_METAB, INPUT);
  pinMode(BTN_DOWN_METAB, INPUT);
  pinMode(BTN_UP_IVEST, INPUT);
  pinMode(BTN_DOWN_IVEST, INPUT);

  pinMode(CHAVE_PENDRIVE, OUTPUT);
  digitalWrite(CHAVE_PENDRIVE, HIGH);

  //led de status do pendrive
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, ledState);

  //essa função só existe pro Arduino DUE
  //antes de compilar, selecione a placa para Arduino Due na interface do Arduino
  analogReadResolution(12); //ADC resolução 12bits 0-4096

  //Inicialização dos displays LCD
  lcd_primeiro.init();
  lcd_primeiro.home();
  lcd_primeiro.backlight();

  lcd_segundo.init();
  lcd_segundo.home();
  lcd_segundo.backlight();

  //inicialização do rtc
  Wire.begin();
  rtc.begin();

  //Monta do cabeçalho no display LCD para os dados do confortimetro
  cabecalho_LCD_confortimetro();

  //inicializacao das rotinas do pendrive
  //pendrive usando Serial3 (pinos RX3 e TX3)
  Serial3.begin(9600);
  //Serial.begin(9600); //usada só para debug
  pinMode(PENDRIVE_SS, INPUT); //uso para ver quando o pendrive está conectado

}

void loop() {
  tempo_ms = millis();

#if 0
  //calcula uma média de 10 amostras, serve como filtro de ruído digital
  tAr_filtrado = calculaMedia(SENSOR_TAR, AMOSTRAS_MEDIA);
  tGlobo_filtrado = calculaMedia(SENSOR_TGLOBO, AMOSTRAS_MEDIA);
  vAr_filtrado = calculaMedia(SENSOR_VEL_AR, AMOSTRAS_MEDIA);
  UR_filtrado = calculaMedia(SENSOR_UR, AMOSTRAS_MEDIA);

  //faz a conversão dos valores filtrados para nivel de tensão 0-3V3
  tAr_filtrado = (tAr_filtrado * 3.3) / 4095.0;//passa para V em double para usar na função de calibração
  tGlobo_filtrado = (tGlobo_filtrado * 3.3) / 4095.0;//passa para V em double para usar na função de calibração
  vAr_filtrado = (vAr_filtrado * 3.3) / 4095.0;//passa para V em double para usar na função de calibração
  UR_filtrado = (UR_filtrado * 3.3) / 4095.0;//passa para V em double para usar na função de calibração
#endif

  //a função calibrar_tensao recebe como parâmetro uma tensão, por isso o passo anterior é necessário
  tAr = calibrar_termistores(tAr_filtrado);
  tGlobo = calibrar_termistores(tGlobo_filtrado);
  vAr = calibrar_termistores(vAr_filtrado);
  UR = calibrar_umidade(UR_filtrado);

  media_tAr += tAr;
  media_tGlobo += tGlobo;
  media_vAr += vAr;
  media_UR += UR;
  contagem_amostras++; 
  
  //variaveis de teste coletadas do programa que o Saulo forneceu
  tAr = 25.1;
  tGlobo = 30.3;
  UR = 60;
  vAr = 1.2;
  PMV = 1.22;
  PPD = 99.8;

  if (digitalRead(BTN_UP_METAB) == HIGH) {
    metabolismo += 2;
    if (metabolismo > 300) metabolismo = 300;
  }
  if (digitalRead(BTN_DOWN_METAB) == HIGH) {
    metabolismo -= 2;
    if (metabolismo < 50) metabolismo = 50;
  }
    if (digitalRead(BTN_UP_IVEST) == HIGH) {
    indice_de_vestimenta += 0.1;
    if (indice_de_vestimenta > 2.0) indice_de_vestimenta = 2.0;
  }
  if (digitalRead(BTN_DOWN_IVEST)) {
    indice_de_vestimenta -= 0.1;
    if (indice_de_vestimenta < 0.1) indice_de_vestimenta = 0.1;
  }
  if (digitalRead(CHAVE_GRAVACAO_1MIN)== HIGH) intervalo_gravacao = UM_MINUTO;
  if (digitalRead(CHAVE_GRAVACAO_10MIN)== HIGH) intervalo_gravacao = DEZ_MINUTOS;
  if (digitalRead(CHAVE_GRAVACAO_60MIN) == HIGH) intervalo_gravacao = UMA_HORA;

  Calcula_PMV(tAr, tGlobo, vAr, UR, metabolismo, indice_de_vestimenta,  &PMV, &PPD);

  //Converte em graus Celsius utilizando a função de SteinHart-Hart e escreve no LCD
  atualiza_LCD_1(tAr, tGlobo, vAr, UR);
  atualiza_LCD_2(metabolismo, indice_de_vestimenta, PMV, PPD);

  pendriveState = debounce(PENDRIVE_SS);

  if (pendriveState == HIGH) { //ao conectar o pendrive o sinal SS fica em nível lógico alto
    digitalWrite(LED_STATUS, ledState);

    if (pd_estadoAtual == desconectado) { //se o pendrive estava anteriormente desconectado, abre um novo arquivo
      contador_conectado = 1;
      Read_Clock();
      memset(arquivo_write, 0, sizeof(arquivo_write)); //limpa o nome do arquivo
      memset(arquivo_append, 0, sizeof(arquivo_append)); 
      memset(tmp, 0, sizeof(tmp));
      strcpy(arquivo_write, "$WRITE "); //nome do arquivo máximo suporta 8 letras
      strcpy(arquivo_append, "$APPEND ");
      if (dia < 10)
        snprintf(tmp, sizeof(tmp), "0%d", dia);
      else
        snprintf(tmp, sizeof(tmp), "%d", dia);
      strcat(arquivo_write, tmp);
      strcat(arquivo_append, tmp);
      memset(tmp, 0, sizeof(tmp));
      if (mes < 10)
        snprintf(tmp, sizeof(tmp), "0%d", mes);
      else
        snprintf(tmp, sizeof(tmp), "%d", mes);
      strcat(arquivo_write, tmp);
      strcat(arquivo_append, tmp);
      memset(tmp, 0, sizeof(tmp));
      if (hora < 10)
        snprintf(tmp, sizeof(tmp), "0%d", hora);
      else
        snprintf(tmp, sizeof(tmp), "%d", hora);
      strcat(arquivo_write, tmp);
      strcat(arquivo_append, tmp);
      if (minuto < 10)
        snprintf(tmp, sizeof(tmp), "0%d", minuto);
      else
        snprintf(tmp, sizeof(tmp), "%d", minuto);
      strcat(arquivo_write, tmp);
      strcat(arquivo_append, tmp);
      strcat(arquivo_write, ".TXT");
      strcat(arquivo_append, ".TXT");
      //Serial.println(arquivo_write);
      //Serial.println(arquivo_append);

      //cria o arquivo no pendrive pra gravar
      flash_data(arquivo_write);
      delay(1000);
      pendrive_primeira_vez = 0;
      pd_estadoAtual = conectado;

    }
    if ((pd_estadoAtual == append) && (pd_UltimoEstado == fechado)) {
      flash_data(arquivo_append);
      delay(500);
      pd_UltimoEstado = append;
    }

    if (pd_estadoAtual == fechado) {
      Serial3.write(26);  // 26 is Control-Z character
      delay(500);
      pd_estadoAtual = append;
      pd_UltimoEstado = fechado;
    }

  } else {
    digitalWrite(LED_STATUS, ledState);
    pendrive_primeira_vez = 0;
    pd_estadoAtual = desconectado;
    if (contador_conectado) {
      digitalWrite(CHAVE_PENDRIVE, LOW);
      delay(100);
      digitalWrite(CHAVE_PENDRIVE, HIGH);
      contador_conectado = 0;
    }
  }

  if ((tempo_ms - intervalo_gravacao_ms) >= intervalo_gravacao) {

    if (pendriveState == HIGH) {
      if ((pd_estadoAtual == conectado) || (pd_estadoAtual == append && pd_UltimoEstado != fechado)) {
        //se for conectado pela primeira vez, escreve o começo da tabela no pendrive
        if (pendrive_primeira_vez == 0) {
          cabecalho_pendrive();
          pendrive_primeira_vez = 1;
        }
        //calcula as médias para gravar no pendrive e gerar o calculo do PMV médio
        tAr = media_tAr/contagem_amostras;
        tGlobo = media_tGlobo/contagem_amostras;
        vAr = media_vAr/contagem_amostras;
        UR = media_UR/contagem_amostras;
        Calcula_PMV(tAr, tGlobo, vAr, UR, metabolismo, indice_de_vestimenta,  &PMV, &PPD);

        Read_Clock();
        //escreve a média dos valores e a média do PMV/PPD
        escreve_pendrive(rtc.getDateStr(), rtc.getTimeStr(), tAr, tGlobo, vAr, UR, metabolismo, indice_de_vestimenta, PMV, PPD);
        contagem_amostras = 0, media_tAr = 0, media_tGlobo = 0, media_vAr = 0, media_UR = 0;
          
        pd_estadoAtual = fechado;
      }
    }

    intervalo_gravacao_ms = tempo_ms;
  }
}

/* parte escrita do LCD que não muda */
void cabecalho_LCD_confortimetro() {
  lcd_primeiro.setCursor(0, 0);
  lcd_primeiro.print("Temp. Ar:"); //9
  lcd_primeiro.setCursor(18, 0);
  lcd_primeiro.print((char) 223); // degree symbol
  lcd_primeiro.print("C");
  lcd_primeiro.setCursor(0, 1);
  lcd_primeiro.print("Temp. Globo:"); //12
  lcd_primeiro.setCursor(18, 1);
  lcd_primeiro.print((char) 223); // degree symbol
  lcd_primeiro.print("C");

  lcd_primeiro.setCursor(0, 2);
  lcd_primeiro.print("Veloc. Ar:"); //11
  lcd_primeiro.setCursor(17, 2);
  lcd_primeiro.print("m/s");

  lcd_primeiro.setCursor(0, 3);
  lcd_primeiro.print("Umidade Rel.:"); //14
  lcd_primeiro.setCursor(19, 3);
  lcd_primeiro.print("%");

  lcd_segundo.setCursor(0, 0); //movimento o cursor do display para coluna 0 (inicio) da linha 0
  lcd_segundo.print("Metab.(W/m2):"); //14
  lcd_segundo.setCursor(0, 1); //movimento o cursor para a coluna 0 (inicio) da linha 1
  lcd_segundo.print("Ind.Vest.(clo):"); //16
  lcd_segundo.setCursor(0, 2); //movimento o cursor do display para coluna 0 (inicio) da linha 0
  lcd_segundo.print("PMV:"); //5
  lcd_segundo.setCursor(0, 3); //movimento o cursor para a coluna 0 (inicio) da linha 1
  lcd_segundo.print("PPD:"); //5
}

//as funções de atualizar o LCD só escrevem no lugar que sobrou, depois de escrever o cabeçalho
//por enquanto está printando tudo como inteiro, em breve arrumar para float  o que precisa
void atualiza_LCD_1(float tempAr, float tempGlobo, float VelAr, uint8_t UmRel) {
  lcd_primeiro.setCursor(10, 0);
  lcd_primeiro.print("       "); //limpa a região antes de escrever
  lcd_primeiro.setCursor(10, 0);
  lcdPrintFloat(tempAr, 1, 1);
  lcd_primeiro.setCursor(13, 1);
  lcd_primeiro.print("    ");
  lcd_primeiro.setCursor(13, 1);
  lcdPrintFloat(tempGlobo, 1, 1);
  lcd_primeiro.setCursor(11, 2);
  lcd_primeiro.print("    ");
  lcd_primeiro.setCursor(11, 2);
  lcdPrintFloat(VelAr, 2, 1);
  lcd_primeiro.setCursor(14, 3);
  lcd_primeiro.print("     ");
  lcd_primeiro.setCursor(14, 3);
  lcd_primeiro.print(UmRel); //escreve a partir da coluna 14
}

void atualiza_LCD_2(uint8_t metabolismo, float ind_vestimenta, float pmv, uint8_t ppd) {
  lcd_segundo.setCursor(14, 0);
  lcd_segundo.print("    ");
  lcd_segundo.setCursor(14, 0);
  lcd_segundo.print(metabolismo); //escreve a partir da coluna 14
  lcd_segundo.setCursor(16, 1);
  lcd_segundo.print("    ");
  lcd_segundo.setCursor(16, 1);
  lcdPrintFloat(ind_vestimenta, 2, 2);
  lcd_segundo.setCursor(5, 2);
  lcd_segundo.print("              ");
  lcd_segundo.setCursor(5, 2);
  lcdPrintFloat(pmv, 1, 2);
  lcd_segundo.setCursor(5, 3);
  lcd_segundo.print("              ");
  lcd_segundo.setCursor(5, 3);
  lcd_segundo.print(ppd); //escreve a partir da coluna 5
}

/* int calculaMedia(double entrada, double n_amostras);
 *  A função retorna a média de n amostragens da entrada especificada
 */
float calculaMedia(float entrada, float n_amostras) {
  int i = 0;
  float media_leituras = 0;

  for (i = 0; i < n_amostras; i++) {
    media_leituras += analogRead(entrada);
  }
  media_leituras /= n_amostras;
  return media_leituras;
}

/* utilizada na representação de números com vírgula no lcd */
//recebe como parametros a variavel, a quantidade de casas depois da virgula, e se é o display 1 ou o 2
void lcdPrintFloat(float val, byte precision, uint8_t numero_display) {
  // prints val on a ver 0012 text lcd with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: lcdPrintFloat( 3.1415, 2); // prints 3.14 (two decimal places)
  if (numero_display == 1) {
    if (val < 0.0) {
      lcd_primeiro.print('-');
      val = -val;
    }

    lcd_primeiro.print((long) val);  //prints the integral part
    if (precision > 0) {
      lcd_primeiro.print("."); // print the decimal point
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
        lcd_primeiro.print("0");
      lcd_primeiro.print(frac, DEC);
      delay(100);
    }
  } else {
    if (val < 0.0) {
      lcd_segundo.print('-');
      val = -val;
    }

    lcd_segundo.print((long) val);  //prints the integral part
    if (precision > 0) {
      lcd_segundo.print("."); // print the decimal point
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
        lcd_segundo.print("0");
      lcd_segundo.print(frac, DEC);
      delay(100);
    }

  }
}

// Create a printing function which has a built-in delay
void flash_data(char *pstring) {
  Serial3.println(pstring);
  delay(200);
}

/* Grava os dados do experimento no pendrive pela interface serial em hardware definida (Serial3)*/
void cabecalho_pendrive() {
  flash_data("Data;Tempo;T_ar;T_globo;Vel.Ar;UR;metabolismo;indice_vestimenta;PMV;PPD");
}

/* Grava os dados do experimento no pendrive pela interface serial em hardware definida (Serial3)*/
void escreve_pendrive(char * data, char* tempo, float tempAr, float tempGlobo, float VelAr, uint8_t UmRel, uint8_t metabolismo, float indice_de_vestimenta, float PMV, uint8_t PPD) {
  memset(string_temp, 0, sizeof(string_temp)); //limpa a string string_temp
  memset(dados_pendrive, 0, sizeof(dados_pendrive)); //limpa a string dados_pendrive
  strcat(dados_pendrive, data);
  strcat(dados_pendrive, ";");
  strcat(dados_pendrive, tempo);
  strcat(dados_pendrive, ";");
  snprintf(string_temp, sizeof(string_temp), "%.1f", tempAr);
  strcat(dados_pendrive, string_temp);
  memset(string_temp, 0, sizeof(string_temp));
  strcat(dados_pendrive, ";");
  snprintf(string_temp, sizeof(string_temp), "%.1f", tempGlobo);
  strcat(dados_pendrive, string_temp);
  memset(string_temp, 0, sizeof(string_temp));
  strcat(dados_pendrive, ";");
  snprintf(string_temp, sizeof(string_temp), "%.1f", VelAr);
  strcat(dados_pendrive, string_temp);
  memset(string_temp, 0, sizeof(string_temp));
  strcat(dados_pendrive, ";");
  snprintf(string_temp, sizeof(string_temp), "%d", UmRel);
  strcat(dados_pendrive, string_temp);
  memset(string_temp, 0, sizeof(string_temp));
  strcat(dados_pendrive, ";");
  snprintf(string_temp, sizeof(string_temp), "%d", metabolismo);
  strcat(dados_pendrive, string_temp);
  strcat(dados_pendrive, ";");
  snprintf(string_temp, sizeof(string_temp), "%.2f", indice_de_vestimenta);
  strcat(dados_pendrive, string_temp);
  strcat(dados_pendrive, ";");
  snprintf(string_temp, sizeof(string_temp), "%.1f", PMV);
  strcat(dados_pendrive, string_temp);
  strcat(dados_pendrive, ";");
  snprintf(string_temp, sizeof(string_temp), "%d", PPD);
  strcat(dados_pendrive, string_temp);
  //gravo a string completa no pendrive
  flash_data(dados_pendrive);
}

//função escrita pelo Saulo com as constantes de calibração dos termistores de 10k (levantada por aproximacao no Excel)
//a função recebe um valor double de tensão e retorna a respectiva temperatura em Celsius referente a esta tensão
float calibrar_termistores(float temperatura_em_volts) {
//constantes calibracao para os termistores Tar, Tglobo e o Var
//esses dados foram disponibilizados pelo Saulo, vieram de uma curva de calibração
const double a = 2.337752299;
const double b = -10.7972788718875;
const double c = 17.5329682071063;
const double d = 20.3911275138112;
const double e = -25.2405251989491;

  float retorno_calibracao;
  retorno_calibracao = a * pow(temperatura_em_volts, 4) + b * pow(temperatura_em_volts, 3) + c * pow(temperatura_em_volts, 2)
      + d * pow(temperatura_em_volts, 1) + e * pow(temperatura_em_volts, 0);
  return retorno_calibracao;
}

//função escrita pelo Saulo com as constantes de calibração do sensor de umidade
//a função recebe um valor double de tensão e retorna a respectiva temperatura em Celsius referente a esta tensão
float calibrar_umidade(float temperatura_em_volts) {
//constantes calibracao para o sensor de umidade Honeywell HIH-5030
//esses dados foram disponibilizados pelo Saulo, vieram de uma curva de calibração
const double f = 47.64627406;
const double g = -23.82075472;

  float retorno_calibracao;
  retorno_calibracao = f * pow(temperatura_em_volts, 1) + g * pow(temperatura_em_volts, 0);
  return retorno_calibracao;
}

/* atualiza os dados de tempo e data de acordo com o RTC */
void Read_Clock() {
  t = rtc.getTime();
  hora = t.hour;
  minuto = t.min;
  segundo = t.sec;
  dia = t.date;
  mes = t.mon;
  ano = t.year;
}

/* função para trocar um caracter (chr) por outro (repl_chr) dentro de uma string */
void str_replace(char *s, char chr, char repl_chr) {
  int i = 0;
  while (s[i] != '\0') {
    if (s[i] == chr) {
      s[i] = repl_chr;
    }
    i++;
  }
  Serial.println(s);
}

  int debounce(int pin) {

    int pinState;
    static int lastPinState = HIGH;   // the previous reading from the input pin

    // the following variables are long's because the time, measured in miliseconds,
    // will quickly become a bigger number than can be stored in an int.
    static long lastDebounceTime = 0;  // the last time the output pin was toggled
    
    long debounceDelay = 30;    // the debounce time; increase if the output flickers      
    int reading;
    
    // read the state of the switch into a local variable:
    reading = digitalRead(pin);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastPinState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != pinState) {
        pinState = reading;
    }
    
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastPinState = reading;
  
  return pinState;  
}


#if 1
/* função que calcula o PMV */
void Calcula_PMV(float Tar, float TGlobo, float VelocAr, float UR, float metab, float indVest, float * PMV, float * PPD) {
  char strin[30], temp[30];
  int cont, i;
  float aa, bb, apa;
  float PMV_A, PMV_B, Esk, Eres, R, C, L, Cres;
//float e double no arduino são 32bit

  float IcloSI, Psat, Pvap, fcl1, fcl2, hc_veloc;
  float hc, new_ClothTemperature, Delta, hc_delT, fcl1_SI, fcl2_SI;
  float Iclo, m, fcl, ClothTemperature;
  
  Iclo = indVest;
  IcloSI = Iclo * 0.155;  // Iclo  em m2K/W  (unidade SI)
              // Iclo em "clo"

  m = metab;  // m em W/m2 , 1 met = 58.82 W/m2

  // The use of casts grants the double format in results

  Psat = (float) 610.78 * exp(Tar / (Tar + 238.3) * 17.2694);

  Pvap = (float) ((float) Psat * UR) / 100.0;

  fcl1_SI = 1.0 + (float) 1.290 * IcloSI;
  fcl2_SI = 1.05 + (float) 0.645 * IcloSI;

  if (IcloSI <= 0.078) {
    fcl = fcl1_SI;
  } else {
    fcl = fcl2_SI;
  }

  if (VelocAr <= 0)
    VelocAr = 0;
  if (VelocAr >= 5)
    VelocAr = 5;

  hc_veloc = 12.1 * sqrt(VelocAr);
  hc = hc_veloc;

  ClothTemperature = (float) (35.7 + Tar) / 2;  // chute inicial
  cont = 0;

  for (i = 0; i < 100; i++) {

    if (ClothTemperature >= 50.0) // para não travar no "pow"
        {
      ClothTemperature = 50.0;
    }

    Delta = abs(ClothTemperature - Tar);
    hc_delT = 2.38 * pow(Delta, 0.25);

    if (hc_delT >= hc_veloc) // sempre pega o maior
      hc = hc_delT;
    else
      hc = hc_veloc;

    aa = 3.96 * pow(10, -8) * fcl * (pow((ClothTemperature + 273.0), 4) - pow((TGlobo + 273), 4));
    bb = fcl * hc * (ClothTemperature - Tar);

    new_ClothTemperature = 35.7 - 0.028 * m - IcloSI * (aa + bb);
    ;

    ClothTemperature = (new_ClothTemperature + 2 * ClothTemperature) / 3;
    // --- isso é feito p/ não divergir. Dá pouco peso à nova temp calculada

    if (ClothTemperature <= 1.0)
      ClothTemperature = 1.0;
    if (ClothTemperature >= 50.0)
      ClothTemperature = 50.0;

    cont = cont + 1;

  }  // fim da iteracao

  PMV_A = 0.303 * exp(-0.036 * m) + 0.028;

  PMV_B = m - (3.05 * 0.001) * (5733 - 6.99 * m - Pvap) - 0.42 * (m - 58.15) - (1.7 * 0.00001) * m * (5867 - Pvap)
      - (0.0014) * m * (34 - Tar) - (3.96E-8) * fcl * (pow((ClothTemperature + 273.0), 4) - pow((TGlobo + 273), 4))
      - fcl * hc * (ClothTemperature - Tar);

  *PMV = PMV_A * PMV_B;

  *PPD = 100.0 - 95.0 * exp(-(0.03353 * pow(*PMV, 4) + 0.2179 * pow(*PMV, 2)));
}
#endif
