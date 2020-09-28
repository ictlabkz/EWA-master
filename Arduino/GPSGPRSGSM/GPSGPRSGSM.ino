#include <SD.h>     // Подключаем библиотеку для работы с SD картой
#include <dht11.h> // подключение библиотеки для работы с цифровым датчиком температуры и влажности
dht11 DHT;               // Объявление переменной класса dht11
#define DHT11_PIN 8
#include <SoftwareSerial.h>
#include <Wire.h>

// объявление датчика BME280 датчик атмосферного давления + аналоговый датчик температуры

#define BME280_ADDRESS 0x76 //I2C (SDA  +  SCL)
unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;
 
uint16_t dig_T1;
 int16_t dig_T2;
 int16_t dig_T3;
uint16_t dig_P1;
 int16_t dig_P2;
 int16_t dig_P3;
 int16_t dig_P4;
 int16_t dig_P5;
 int16_t dig_P6;
 int16_t dig_P7;
 int16_t dig_P8;
 int16_t dig_P9;
 int8_t  dig_H1;
 int16_t dig_H2;
 int8_t  dig_H3;
 int16_t dig_H4;
 int16_t dig_H5;
 int8_t  dig_H6;


SoftwareSerial gsm(2, 3); // RX, TX
String _response = "";


File myFile;
int hydrogenA0 = A0;
int smokeA2 = A1;
int alcoA3 = A3;


void setup()
{
  
    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 3;               //Normal mode
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off 
    uint8_t spi3w_en = 0;           //3-wire SPI Disable
    
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;
    
    Wire.begin();
    
    writeReg(0xF2,ctrl_hum_reg);
    writeReg(0xF4,ctrl_meas_reg);
    writeReg(0xF5,config_reg);
    readTrim();       

  
  // put your setup code here, to run once:


pinMode(hydrogenA0,INPUT);
pinMode(smokeA2,INPUT);
pinMode(alcoA3,INPUT);
  
  // Open serial communications to computer
  Serial.begin(115200);
  gsm.begin(115200); // Default for the board

  sendATCommand("AT", true); // Отправили AT для настройки скорости обмена данными
  _response = sendATCommand("AT+CMGF=1;&W", true); // Включаем текстовый режима SMS (Text mode) и сразу сохраняем значение (AT&W)


  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  

}

String sendATCommand(String cmd, bool waiting) {
  String _resp = ""; // Переменная для хранения результата
  Serial.println(cmd); // Дублируем команду в монитор порта
  gsm.println(cmd); // Отправляем команду модулю
  if (waiting) { // Если необходимо дождаться ответа...
    _resp = waitResponse(); // ... ждем, когда будет передан ответ
    // Если Echo Mode выключен (ATE0), то эти 3 строки можно закомментировать
    if (_resp.startsWith(cmd)) { // Убираем из ответа дублирующуюся команду
      _resp = _resp.substring(_resp.indexOf("\r", cmd.length()) + 2);
    }
    Serial.println(_resp); // Дублируем ответ в монитор порта
  }
  return _resp; // Возвращаем результат. Пусто, если проблема
}


String waitResponse() { // Функция ожидания ответа и возврата полученного результата
  String _resp = ""; // Переменная для хранения результата
  long _timeout = millis() + 10000; // Переменная для отслеживания таймаута (10 секунд)
  while (!gsm.available() && millis() < _timeout) {}; // Ждем ответа 10 секунд, если пришел ответ или наступил таймаут, то...
  if (gsm.available()) { // Если есть, что считывать...
    _resp = gsm.readString(); // ... считываем и запоминаем
  }
  else { // Если пришел таймаут, то...
    Serial.println("Timeout..."); // ... оповещаем об этом и...
  }
  return _resp; // ... возвращаем результат. Пусто, если проблема }
}

void loop() {


  // сбор данных с датчика BME280

    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;
    
    readData();
    
    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / 100.0;
    hum_act = (double)hum_cal / 1024.0;
    

  if (gsm.available()) { // Если модем, что-то отправил...
    _response = waitResponse(); // Получаем ответ от модема для анализа
    _response.trim(); // Убираем лишние пробелы в начале и конце
    Serial.println(_response); // Если нужно выводим в монитор порта
    //....
    if (_response.startsWith("+CMGS:")) {       // Пришло сообщение об отправке SMS
      int index = _response.lastIndexOf("\r\n");// Находим последний перенос строки, перед статусом
      String result = _response.substring(index + 2, _response.length()); // Получаем статус
      result.trim();   // Убираем пробельные символы в начале/конце

      if (result == "OK") {                     // Если результат ОК - все нормально
        Serial.println ("Message was sent. OK");
      }
      else {                                    // Если нет, нужно повторить отправку
        Serial.println ("Message was not sent. Error");
      }
    }
  }

  if (Serial.available()) { // Ожидаем команды по Serial...
    gsm.write(Serial.read()); // ...и отправляем полученную команду модему
  };

DHT.read(DHT11_PIN);
String str1 = String (DHT.temperature);
Serial.print("Temp = ");
Serial.println(str1);

String str2 = String (DHT.humidity);
Serial.print("Hum = ");
Serial.println(str2);

/*//Выводим показания датчика водорода
String str3 = String (analogRead(hydrogenA0));
Serial.print("Hydrogen = ");
Serial.println(str3);*/

//Выводим показания датчика атм давления
String str4 = String (press_act);
Serial.print("DegC PRESS = ");
Serial.println(str4);

//Выводим показания датчика газа
String str5 = String (analogRead(smokeA2));
Serial.print("HydrocarbonGases = ");
Serial.println(str5);

// open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("measure.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {

  myFile.print("Temperature: ");
  myFile.println(str1);

  myFile.print("Humidity: ");
  myFile.println(str2);

 /* myFile.print("Hydrogen: ");
  myFile.println(str3);*/

  myFile.print("DegC PRESS: ");
  myFile.println(str4);

  myFile.print("Smoke&HydrocarbonGases: ");
  myFile.println(str5);
  
    // close the file:
    myFile.close();
  }


sendSMS("+77778983735", "Temp" + str1);
delay(30000);
sendSMS("+77778983735", "Mbar" + str4);
delay(30000);
  
}

void sendSMS(String phone, String message)
{
  sendATCommand("AT+CMGS=\"" + phone + "\"", true);             // Переходим в режим ввода текстового сообщения
  sendATCommand(message + "\r\n" + (String)((char)26), true);   // После текста отправляем перенос строки и Ctrl+Z
}


//библиотека для работы с датчиком BME280

void readTrim()
{
    uint8_t data[32],i=0;
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,24);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xA1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,1);
    data[i] = Wire.read();
    i++;
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,7);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;    
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];   
}
void writeReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();    
}
 
 
void readData()
{
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,8);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}
 
 
signed long int calibration_T(signed long int adc_T)
{
    
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}
 
unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}
 
unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;
    
    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);   
}
