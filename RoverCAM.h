// ArduCAM Mini demo (C)2017 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use the enhanced functions
// This demo was made for ArduCAM_Mini_5MP_Plus.
// It can  continue shooting and store it into the SD card  in JPEG format
// The demo sketch will do the following tasks
// 1. Set the camera to JPEG output mode.
// 2. Capture a JPEG photo and buffer the image to FIFO
// 3.Write the picture data to the SD card
// 5.close the file
//You can change the FRAMES_NUM count to change the number of the picture.
//IF the FRAMES_NUM is 0X00, take one photos
//IF the FRAMES_NUM is 0X01, take two photos
//IF the FRAMES_NUM is 0X02, take three photos
//IF the FRAMES_NUM is 0X03, take four photos
//IF the FRAMES_NUM is 0X04, take five photos
//IF the FRAMES_NUM is 0X05, take six photos
//IF the FRAMES_NUM is 0X06, take seven photos
//IF the FRAMES_NUM is 0X07, continue shooting until the FIFO is full
//You can see the picture in the SD card.
// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM_Mini_5MP_Plus
// and use Arduino IDE 1.6.8 compiler or above
//#include <Wire.h>
#include <ArduCAM.h>
//#include <SPI.h>
//#include <SD.h>
#include "memorysaver.h"
//This demo can only work on OV5640_MINI_5MP_PLUS or OV5642_MINI_5MP_PLUS platform.
#if !(defined (OV5640_MINI_5MP_PLUS)||defined (OV5642_MINI_5MP_PLUS))
#error Please sel libect the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define   FRAMES_NUM    0x02
// set pin 5 as the slave select for the digital pot:
const int CS = 5;
//pin 10 as slave select for SD card
#define SD_CS 10
bool is_header = false;
int total_time = 0;
#if defined (OV5640_MINI_5MP_PLUS)
ArduCAM myCAM(OV5640, CS);
#else
ArduCAM myCAM(OV5642, CS);
#endif
uint8_t read_fifo_burst(ArduCAM myCAM, int filenumber);


void setup_rovercam() {

  // put your setup code here, to run once:
  uint8_t vid, pid;
  uint8_t temp;
#if defined(__SAM3X8E__)
  Wire1.begin();
#else
  Wire.begin();
#endif
  Serial.println(F("ArduCAM Start!"));
  // set the CS as an output:
  pinMode(CS, OUTPUT);

  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55)
    {
      Serial.println(F("SPI interface Error!"));
      delay(1000); continue;
    } else {
      Serial.println(F("SPI interface OK.")); break;
    }
  }
#if defined (OV5640_MINI_5MP_PLUS)
  while (1) {
    //Check if the camera module type is OV5640
    myCAM.rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x40)) {
      Serial.println(F("Can't find OV5640 module!"));
      delay(1000); continue;
    } else {
      Serial.println(F("OV5640 detected.")); break;
    }
  }
#else
  while (1) {
    //Check if the camera module type is OV5642
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x42)) {
      Serial.println(F("Can't find OV5642 module!"));
      delay(1000); continue;
    } else {
      Serial.println(F("OV5642 detected.")); break;
    }
  }
#endif
  //Initialize SD Card
  /*  while (!SD.begin(SD_CS))
    {
      Serial.println(F("SD Card Error!")); delay(1000);
    }
    Serial.println(F("SD Card detected."));
  */
  //Change to JPEG capture mode and initialize the OV5640 module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
  myCAM.clear_fifo_flag();
  myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
}
bool capture(int filenumber) {
  // put your main code here, to run repeatedly:
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
#if defined (OV5640_MINI_5MP_PLUS)
  myCAM.OV5640_set_JPEG_size(OV5640_320x240); delay(1000);
#else
  myCAM.OV5642_set_JPEG_size(OV5642_320x240); delay(1000);
#endif
  //Start capture
  myCAM.start_capture();
  Serial.println(F("start capture."));
  total_time = millis();
  while ( !myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  Serial.println(F("CAM Capture Done."));
  total_time = millis() - total_time;
  Serial.print(F("capture total_time used (in miliseconds):"));
  Serial.println(total_time, DEC);
  total_time = millis();
  read_fifo_burst(myCAM, filenumber);
  total_time = millis() - total_time;
  Serial.print(F("save capture total_time used (in miliseconds):"));
  Serial.println(total_time, DEC);
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
  delay(5000);
}
uint8_t read_fifo_burst(ArduCAM myCAM, int filenumber)
{
  myCAM.OV5642_set_JPEG_size(OV5642_320x240); delay(1000);

  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  static int i = 0;
  static int k = 0;
  char str[8];
  File outFile;
  byte buf[256];
  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //8M
  {
    Serial.println("Over size.");
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("Size is 0."));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  i = 0;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);
      //Close the file
      outFile.close();
      Serial.println(F("OK"));
      is_header = false;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      myCAM.CS_HIGH();
      //Create a avi file
      itoa(filenumber, str, 10);
      strcat(str, ".jpg");
      //Open the new file
      outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
      if (! outFile)
      {
        Serial.println(F("File open failed"));
        return 0;
        //while (1);
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }
  myCAM.CS_HIGH();
  return 1;
}

//helper to print char as two digit hex with trailing space ;)
void p(char X) {
  if (X < 16) {
    Serial.print("0");
  }

  Serial.print(X, HEX);
  Serial.print(" ");
}

//char symbols[] = { '@', '%', '#', 'x', '+', '=', ':', '-', '.', ' ' };

char symbols[] = { '.', ',', ':', ';', 'i', 'r', 's', 'X', 'A', '2', '5', '3', 'h', 'M', 'H', 'G', 'S', '#', '9', 'B', '&', '@'};


void p_ascii(char X) {
  Serial.write(symbols[(int) X / 12]);
}
void p_binary(char X) {
  char buf[9];
  for (int i = 0; i < 8; i++)
  {

    if (X & 1)
      buf[8 - i] = '1';
    else
      buf[8 - i] = '0';
    X >>= 1;

  }
  Serial.write(buf);
  Serial.write(' ');
}

boolean raw_first_time = true;

float fifo_burst_contrast(ArduCAM myCAM, boolean preview)
{
  uint8_t resolution = OV5642_640x480;
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.OV5642_set_RAW_size(resolution); delay(10);

  unsigned long start =  millis();

  myCAM.start_capture();
  //delay(50);
  //while(myCAM.read_fifo()==0 && myCAM.read_fifo()==0) {delay(50);}
  //delay(50);

  while ( !myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  int line = 480;//640;
  int column = 640;//480;these were bacwards wtf
  if (preview)
  {
    Serial.print("Startup took: ");
    Serial.println(millis() - start);
    start = millis();
  }

  const int i_n = 16;
  const int j_n = 16;
  const int hor_start = 200;

  char VL;
  int sub_area[i_n * 2][j_n * 2];
  for (int i = 0; i < i_n * 2; i++)
  {
    for (int j = 0; j < column; j++)
    {
      VL = myCAM.read_fifo();
      if (j >= hor_start && j < hor_start + 2 * j_n && i >= 0 && i < 2 * i_n)
      {
        sub_area[i - 0][j - hor_start] = VL;
        if (preview)
        {
          Serial.print((int)VL);
          Serial.print(" ");
        }
      }
    }
    if (i >= 0 && i < i_n * 2 && preview)
      Serial.write('\n');
  }
  if (preview)
  {
    Serial.print("Image readover took: ");
    Serial.println(millis() - start);
    start = millis();
    Serial.println("Downsized image: ");
  }

  float reduced[i_n][j_n];
  float avg = 0;
  for (int i = 0; i < i_n; i++)
  {
    for (int j = 0; j <  j_n; j++)
    {
      //average downsample by 4
      reduced[i][j] = float((sub_area[2 * i][2 * j] + sub_area[2 * i + 1][2 * j] + sub_area[2 * i][2 * j + 1] + sub_area[2 * i + 1][2 * j + 1]));
      avg += reduced[i][j];
      if (preview)
      {
        Serial.print(reduced[i][j]/ (4 * 255));
        Serial.print(" ");
      }

    }
    if (preview)
      Serial.write('\n');
  }
  avg = avg / (i_n * j_n);
  float total_diff = 0;
  /*
    float filter[3][3] =
    { {0.125, 0.125, 0.125},
    {0.125, -1,  0.125},
    {0.125, 0.125, 0.125}
    };
  */
  float f = 1 / 24;
  float filter[5][5] =
  { {f, f, f, f, f},
    {f, f, f, f, f},
    {f, f, -1, f, f},
    {f, f, f, f, f},
    {f, f, f, f, f}
  };

  int filter_size = 5;
  if (preview)
    Serial.println("Filter sums:");
  for (int i = 0; i < i_n - filter_size + 1; i++)
  {
    for (int j = 0; j < j_n - filter_size + 1; j++)
    {
      float filter_sum = 0;
      for (int filter_i = 0; filter_i <  filter_size; filter_i++)
      {
        for (int filter_j = 0; filter_j <  filter_size; filter_j++)
        {
          filter_sum += reduced[i + filter_i][j + filter_j] * filter[filter_i][filter_j] ;
          //total_diff += sq(reduced[i][j] - avg);
        }
      }
      if (preview)
      {
        Serial.print(i_n * j_n * sq((filter_sum/ (4 * 255)) / (sq(filter_size))));
        Serial.write(' ');
      }
      total_diff += sq((filter_sum/ (4 * 255)) / (sq(filter_size)));
    }
    if (preview)
      Serial.println();
  }
  if (preview)
  {
    Serial.print("Avg: ");
    Serial.println(avg);

    Serial.print("Processing took:");
    Serial.println(millis() - start);
    start = millis();
  }
  //Serial.print(total_diff);

  myCAM.clear_fifo_flag();



  return total_diff;

}

