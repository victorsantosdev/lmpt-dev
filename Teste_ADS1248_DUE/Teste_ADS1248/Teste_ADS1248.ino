
#include <SPI.h> //necessário para o ADS1248
#include <math.h>

/*********** definições de constantes do programa *******************/
#define SERIAL_BAUDRATE 9600
#define ADS1248_NCANAIS 4
#define ADS1248_CADENCIA 1000

// set pin 10 as the slave select for the digital pot:

const uint8_t LED_STATUS = 13; //led da própria placa
const uint8_t ADS_SLAVE_SELECT = 52;//Instead of one Chip Select (SS) pin, the Due supports three.  These are hardware Digital Pin 10 (SPI-CS0), Digital Pin 4 (SPI-CS1) and Digital 52 (SPI-CS2).  
//These pin values are used in the Due SPI library calls to determine which hardware SPI device you are addressing (up to three at the same time).


long long ads1248_cadencia_ms = 0;
long long tempo_ms = 0;

//variaveis do ads1248 e dos termistores
double ads1248_ganhoCanal[4]; //vetor de configuração de ganho por canal
double ads1248_ad[4]; //4 canais com 8 entradas diferenciais
double q_sup = 0, q_inf = 0;

//constantes definidas por Saulo Guths
#define C_SUP 0.115
#define C_INF 0.117

void setup() {

  //Serial que se comunica com o software no PC
  Serial.begin(SERIAL_BAUDRATE);

  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  ads1248_ganhoCanal[0] = 4;  //2 elev 4 = 16 (datasheet)
  ads1248_ganhoCanal[1] = 4;
  ads1248_ganhoCanal[2] = 4;
  ads1248_ganhoCanal[3] = 4;

  //SPI
  pinMode(ADS_SLAVE_SELECT, OUTPUT);
  digitalWrite(ADS_SLAVE_SELECT, HIGH);
  
  SPI_init();
  ads1248_clear();
}

void loop() {
  tempo_ms = millis();

  if ((tempo_ms - ads1248_cadencia_ms) >= ADS1248_CADENCIA) {
    double ads_canal0 = ads1248_ler_amplificador(0);
    delay(50);
    double ads_canal1 = ads1248_ler_amplificador(1);
    delay(50);
    double ads_canal2 = ads1248_ler_amplificador(2);
    delay(50);
    double ads_canal3 = ads1248_ler_amplificador(3);
    delay(50);
    if ((isinf(ads_canal0) != 1) || (isnan(ads_canal0) != 1))  {
    ads1248_ad[0] = ads_canal0;
    delay(5);
    }
    if ((isinf(ads_canal1) != 1) || (isnan(ads_canal1) != 1)) {
    ads1248_ad[1] = ads_canal1;
    delay(5);
    }
    if ((isinf(ads_canal2) != 1) || (isnan(ads_canal2) != 1)) {    
    ads1248_ad[2] = ads_canal2;
    delay(5);
    }
    if ((isinf(ads_canal3) != 1) || (isnan(ads_canal3) != 1)) {
    ads1248_ad[3] = ads_canal3;
    delay(5);
    }

    ads1248_cadencia_ms = tempo_ms;
  }

 
    //Serial Monitor
    
    Serial.print("Canal 1: ");
    Serial.print(ads1248_ad[0]);
    Serial.println("");
    Serial.print("Canal 2: ");
    Serial.print(ads1248_ad[1]);
    Serial.println("");
    Serial.print("Canal 3: ");
    Serial.print(ads1248_ad[2]);
    Serial.println("");
    Serial.print("Canal 4: ");
    Serial.print(ads1248_ad[3]);
    Serial.println("");
   

}

/* Init SPI interface*/
void SPI_init() {
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV84); //84
  //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
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

bool ads1248_ajustar_canal_ganho(int MUX_SP, int MUX_SN, int PGA)
{
  int BCS, DOR;
  int MUX0_word_write, SYS0_word_write, MUX0_word_read, SYS0_word_read;
  int contador_tentativas;
  bool registro_MUX0_ok, registro_SYS0_ok;
  registro_MUX0_ok = false;
  registro_SYS0_ok = false;
  contador_tentativas = 0;
  while (registro_MUX0_ok == false && contador_tentativas < 10) 
  {
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
    if (MUX0_word_write == MUX0_word_read)      registro_MUX0_ok = true;
    contador_tentativas++;
  }
  contador_tentativas = 0;
  while (registro_SYS0_ok == false && contador_tentativas < 10) 
  {
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
    if (SYS0_word_write == SYS0_word_read)      registro_SYS0_ok = true;
    contador_tentativas++;
  }
  if (registro_MUX0_ok && registro_SYS0_ok) 
  {
    return true;
  }
  else {
    return false;
  }
}


double ads1248_ler(int ganho) 
{
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
    tensao_ads1248[leituras] = tensao_ads1248[leituras] * 2.048/8388607;
    if ((MSB >> 7) == 1)      tensao_ads1248[leituras] = (tensao_ads1248[leituras] - 2.048) / pow(2, ganho);
    if ((MSB >> 7) == 0)      tensao_ads1248[leituras] = (tensao_ads1248[leituras]) / pow(2, ganho);
    media_tensao_ads1248 = media_tensao_ads1248 + tensao_ads1248[leituras];
  }
  ///media_tensao_ads1248_corrigida = media_tensao_ads1248; /// ALTERADO
  media_tensao_ads1248 = media_tensao_ads1248 / numero_leitura_media;
  for ( int leituras = 1; leituras <= numero_leitura_media; leituras++)
  {
    if ( tensao_ads1248[leituras] >= (media_tensao_ads1248 - media_tensao_ads1248*0.1) && tensao_ads1248[leituras] <= (media_tensao_ads1248 + media_tensao_ads1248*0.1))
    {
      media_tensao_ads1248_corrigida = media_tensao_ads1248_corrigida + tensao_ads1248[leituras];
      contador_tensoes_ok++;
    }
  }
  if ( contador_tensoes_ok != 0)
  {
    media_tensao_ads1248_corrigida = media_tensao_ads1248_corrigida/contador_tensoes_ok;
    return media_tensao_ads1248_corrigida;
  }
  else
  {
    return media_tensao_ads1248;
  }
}


