/* MIT License
Copyright (c) 2019 Kamicaze
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 -------------------
 Flavio Müller, Kamicaze
 e-mail:  flavio.flmf@gmail.com
 */

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(2, 3);

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ; //Será desconsiderado neste sistema

Kalman kalmanAX; // Create the Kalman instances
Kalman kalmanAY;
Kalman kalmanAZ; //Será desconsiderado neste sistema


/*IMU Data*/
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double accXangle, accYangle, accZangle; // Cálculo de ângulo usando apenas o acelerometro
double gyroXangle, gyroYangle, gyroZangle; // Cálculo de ângulo usando apenas o giroscópio
double compAngleX, compAngleY, compAngleZ; // Ângulo calculado usando um filtro complementar
double kalAngleX, kalAngleY, kalAngleZ; // Ângulo calculado usando um filtro Kalman

double kalX, kalY, kalZ; // Para alguma coisa

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() 
{
  Serial.begin(9600);//
  Wire.begin();
  
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) 
  {
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(200); // Wait for sensor to stabilize

  /*1º - Leitura dos dados Acc XYZ*/
  while (i2cRead(0x3B, i2cData, 6));
  
  /*2º - Organiza os dados de Acc XYZ*/
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  
  /*3º - Calculo de Pitch, Roll e Yaw*/
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    double yaw = atan(accZ / sqrt(accX * accX + accY * accY)) * RAD_TO_DEG;
  
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    double yaw = atan(accZ / sqrt(accX * accX + accY * accY)) * RAD_TO_DEG;
  #endif
  
  /*4º - Inicialização do Filtro de Kalman*/
  kalmanX.setAngle(roll); // Definir ângulo inicial
  kalmanY.setAngle(pitch); // Definir ângulo inicial
  kalmanZ.setAngle(yaw); // Definir ângulo inicial
  
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;

  timer = micros();
}

void loop() 
{
  /* Atualize todos os valores */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
   double yaw = atan(accZ / sqrt(accX * accX + accY * accY)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
   double yaw = atan(accZ / sqrt(accX * accX + accY * accY)) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Converte para deg/s
  double gyroYrate = gyroY / 131.0; // Converte para deg/s
  double gyroZrate = gyroZ / 131.0; // Converte para deg/s

  #ifdef RESTRICT_PITCH
    //X Isso corrige o problema de transição quando o ângulo do acelerômetro salta entre -180 e 180 graus
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    }
    else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; //Taxa de inversão, para que se ajuste à leitura restrita do acelerômetro
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    #else
    
    
    
    //Y Isso corrige o problema de transição quando o ângulo do acelerômetro salta entre -180 e 180 graus
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
    {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    }
    else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calcular o ângulo usando um filtro Kalman

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; //Taxa de inversão, para que se ajuste à leitura restrita do acelerômetro
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calcular o ângulo usando um filtro Kalman


    //Z Isso corrige o problema de transição quando o ângulo do acelerômetro salta entre -180 e 180 graus
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90))
    {
      kalmanZ.setAngle(yaw);
      compAngleZ = yaw;
      kalAngleZ = yaw;
      gyroZangle = yaw;
    }
    else
      kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); //Calcular o ângulo usando um filtro Kalman
  #endif


  gyroXangle += gyroXrate * dt; //Calcular o ângulo do giroscópio sem nenhum filtro
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; //Calcular o ângulo do giroscópio usando a taxa imparcial
  //gyroYangle += kalmanY.getRate() * dt;
  //gyroZangle += kalmanZ.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; //Calcular o ângulo usando um filtro complementar
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  //Redefina o ângulo do giroscópio quando ele deriva demais
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;

  /* Print Data */
#if 0 // Set to 1 to activate
//Dados Brutos
  Serial.print(accX); Serial.print("\t");
  Serial.print("Ax"); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print("Ay"); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");
  Serial.print("Az"); Serial.print("\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print("Gx"); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print("Gy"); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");
  Serial.print("Gz"); Serial.print("\t");

  Serial.print("\t");
#endif


#if 0 // Set to 1 to activate
//Convertido em g
  double Ax = accX/16384;
  double Ay = accY/16384;
  double Az = accZ/16384;

  Serial.print(Ax); Serial.print("\t");
  Serial.print("Ax"); Serial.print("\t");
  Serial.print(Ay); Serial.print("\t");
  Serial.print("Ay"); Serial.print("\t");
  Serial.print(Az); Serial.print("\t");
  Serial.print("Az"); Serial.print("\t");
  Serial.print("\t");
#endif

#if 1 // Set to 1 to activate
//Convertido em m/s²
  double Ax = (accX/16385)*9.8;
  double Ay = (accY/16385)*9.8;
  double Az = (accZ/16385)*9;

  Serial.print(Ax); Serial.print(" m/s² \t");
  Serial.print("Ax"); Serial.print("\t");
  Serial.print(Ay); Serial.print(" m/s² \t");
  Serial.print("Ay"); Serial.print("\t");
  Serial.print(Az); Serial.print(" m/s² \t");
  Serial.print("Az"); Serial.print("\t");
  Serial.print("\t");
#endif


#if 0 // Set to 1 to activate
  Serial.print(roll); Serial.print(" rool \t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print(" comp X \t");
  Serial.print(kalAngleX); Serial.print(" Kal X \t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print(" pitch \t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print(" comp Y \t");
  Serial.print(kalAngleY); Serial.print(" Kal Y \t");
#endif

#if 0 // Set to 1 to activate
  bluetooth.print(Ax); bluetooth.println("\t");
  bluetooth.print("Ax"); bluetooth.println("\t");
  bluetooth.print(Ay); bluetooth.println("\t");
  bluetooth.print("Ay"); bluetooth.println("\t");
  bluetooth.print(Az); bluetooth.println("\t");
  bluetooth.print("Az"); bluetooth.println("\t");
  bluetooth.print(kalAngleX); bluetooth.println("\t");
  bluetooth.print(kalAngleY);bluetooth.println("\t");
#endif

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");
  double temperature = (double)tempRaw / 340.0 + 36.53;//Equação da temperatura em Cº de acordo com o datasheet
  Serial.print(temperature); Serial.print("\t");
#endif
Serial.print("\r\n");
delay(2);
}
