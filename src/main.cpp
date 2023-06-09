#include <mbed.h>

// LCD display lib and font lib
#include "TextDisplay.h"
#include "SPI_TFT_ILI9341.h"
#include "Arial24x23.h"
#include "Arial12x12.h"
#include "Arial10x10.h"
#include "embed_project_log.h"

#define PI 3.1415926

#define CALIBRATION_DATA_LENGTH 300
#define NOISE_THRESHOLD 2
#define DISPLAY_TIME_INTERVAL_MS 1000
#define TWENTY_SECOND_MS 20000
#define DISPLAY_CENTER_X 50
#define DISPLAY_POS_Y 100
#define AXIS_START_X 30
#define AXIS_START_Y 250

#define FIRST_20_SECOND_DATA_NUM 40
#define LCD_DISPLAY_Y_DATA_FACTOR 5
#define LCD_DISPLAY_X_DATA_FACTOR 3
#define LCD_DISPLAY_AXIS_FACTOR 15


#define READ_OK 0
#define READ_ERROR -1
#define UNSET_FLAG -1

// 429 board controls this motion sensor through the SPI interface(Page 19, 429 datasheet).  
SPI g_spi(PF_9, PF_8, PF_7);
// (Page 24 in 429datasheet, gyro sensor's cs is PC1)
DigitalOut g_gyroCs(PC_1);

SPI_TFT_ILI9341 g_LCDController(PF_9, PF_8, PF_7, PC_2, PB_4, PD_13); // mosi, miso, sclk, cs, reset, dc


Timer g_processDataTimer;
int16_t g_sampleInterval;
int16_t g_first20SecDistance;
int16_t g_outputTimer;
double g_distanceinFirst20Sec[FIRST_20_SECOND_DATA_NUM] = {0.0};
int16_t g_timeFirst20Sec[FIRST_20_SECOND_DATA_NUM] = {0};
int8_t g_figureIndex = 0;

// Start L3G4250D sensor to normal mode, which is setting the pd control register 1·
void InitGyroSensor()
{
    g_gyroCs = 0;
    // pd bit is Bit3, set to 1(switch from power down to normal)
    // enable all three axises(Xen, Yen, Zen = 1)
    // What every byte means refer to Page 26 in I3G4250D datasheet
    g_spi.write(0x20);
    g_spi.write(0b11001111);
    g_gyroCs = 1;
}

void DisplayStartPage()
{
    g_LCDController.locate(0, DISPLAY_POS_Y);
    g_LCDController.printf("Author:\n");
    g_LCDController.locate(0, DISPLAY_POS_Y + 20);
    g_LCDController.printf("Lin Yuan");
    g_LCDController.locate(0, DISPLAY_POS_Y + 40);
    g_LCDController.printf("Xinyu Zhang");
    g_LCDController.set_font((unsigned char*)Arial12x12);
    g_LCDController.locate(0, DISPLAY_POS_Y + 70);
    g_LCDController.printf("Important Instruction: ");
    g_LCDController.locate(0, DISPLAY_POS_Y + 90);
    g_LCDController.printf("Put the board on your knee, with the display towards the direction you move");
    wait_us(7000000);
    g_LCDController.set_font((unsigned char*) Arial24x23);
    g_LCDController.locate(0, DISPLAY_POS_Y + 150);
    g_LCDController.printf("Initializing");
    g_LCDController.locate(0, DISPLAY_POS_Y + 180);
    g_LCDController.printf("Remain Still");
}

void InitDisplay()
{
    g_LCDController.set_orientation(0);
    g_LCDController.background(BLACK);
    g_LCDController.foreground(WHITE);
    g_LCDController.cls(); // clear the screen
    g_LCDController.set_font((unsigned char*) Arial24x23);
    DisplayStartPage();
}

void DisplayAxis()
{
    g_LCDController.set_font((unsigned char*) Arial10x10);
    g_LCDController.locate(AXIS_START_X + 80, AXIS_START_Y + 10);
    g_LCDController.printf("Time(s)");      // x lable
    g_LCDController.set_orientation(3);
    g_LCDController.locate(AXIS_START_X + 80, 5);
    g_LCDController.printf("Distance(m)");      // y lable

    
    g_LCDController.set_orientation(0);
    g_LCDController.line(AXIS_START_X, AXIS_START_Y, AXIS_START_X, 100, WHITE);      // y-axis
    g_LCDController.line(AXIS_START_X, AXIS_START_Y, 205, AXIS_START_Y, WHITE);     // x-axis
 
    // Display the tick mark on the axis
    for (int i = 1; i <= 10; i++){
        g_LCDController.line(AXIS_START_X,
            AXIS_START_Y - i * LCD_DISPLAY_AXIS_FACTOR,
            AXIS_START_X - LCD_DISPLAY_Y_DATA_FACTOR,
            AXIS_START_Y - i * LCD_DISPLAY_AXIS_FACTOR, WHITE);
        g_LCDController.line(AXIS_START_X + i * LCD_DISPLAY_AXIS_FACTOR,
            AXIS_START_Y,
            AXIS_START_X + i * LCD_DISPLAY_AXIS_FACTOR,
            AXIS_START_Y + LCD_DISPLAY_Y_DATA_FACTOR, WHITE);
        }
}

int8_t ReadGyroXData(int16_t *readRes)
{
    const int8_t startAddr = 0x28;
    const int8_t readFlag = 0x80;
    const int8_t incrementFlag = 0x40;
    g_gyroCs = 0;
    // Check Bit 3 in status register, if 1 then data is ready
    g_spi.write(readFlag | 0x27);
    int status = g_spi.write(0x00);
    g_gyroCs = 1;
    EMBED_PROJECT_LOG("Status register value: %d\n", status);
    // Data is ready
    if (status & 0b00001000) {
        int8_t lowParts;
        int8_t highParts;
        g_gyroCs = 0;
        g_spi.write(readFlag | incrementFlag | startAddr);
        lowParts = g_spi.write(0x00);
        highParts = g_spi.write(0x00);
        *readRes = (highParts << 8) + lowParts;
        g_gyroCs = 1;

        // Stop the timer and store the timer value( which indicite time esaped since the last successfully read)
        g_processDataTimer.stop();
        g_sampleInterval = g_processDataTimer.read_ms();
        EMBED_PROJECT_LOG("Time interval: %d\n", g_sampleInterval);
        g_processDataTimer.reset();
        g_processDataTimer.start();

        return READ_OK;
    }
    return READ_ERROR;
}

void GetCalibrateData(int32_t *caliData)
{
    // Once powered up, read CALIBRATION_DATA_LENGTH datas and calculate their mean value as base.
    int16_t gyroXData;
    for (int i = 0; i < CALIBRATION_DATA_LENGTH;) {
        if (ReadGyroXData(&gyroXData) == READ_OK) {
            (*caliData) += gyroXData;
            i++;
        }
    }
    *caliData = *caliData / CALIBRATION_DATA_LENGTH;
}

void ProcessData(int16_t data, int32_t caliData, double *distanceData)
{
    double realMeaningData;
    // Rreal = Rraw - Rcali 
    data -= caliData;
    if (data > 0) {
        data = 0;
    }
    data = abs(data);  //transfer to positive

    // According to page 10, 33 of the datasheet of gyroscope, default FS = 245dps/LSB and 
    // typical factor to transfer raw sensor data to real data for +25 Celsius is 8.75 mdps/least significant bit.
    realMeaningData = (0.00875 * data);
    // If the data is within a very small threshold, we consider the board as still.
    if (abs(realMeaningData) < NOISE_THRESHOLD) {
        realMeaningData = 0.0;
    }

    // Integrating velocity to get position.
    // position += Time interval between two sample * (anglar velocity/360)*2PI * radius 
    (*distanceData) += (((double)g_sampleInterval)/ 1000) * (realMeaningData / 360) * 2 * PI * 1.55;
    if (g_outputTimer < TWENTY_SECOND_MS) {
        printf("time interval: %d, angular velocity: %d, outputdata: %d\n", g_sampleInterval, 
            (int16_t)((double)data * 0.00875), (int16_t)*distanceData);
        g_outputTimer += g_sampleInterval;
    }
    EMBED_PROJECT_LOG("time interval: %d, angular velocity: %d, outputdata: %d\n", g_sampleInterval, 
         (int16_t)((double)data * 0.00875), (int16_t)*distanceData);
    
    // Record the distance data for the first 20 seconds, which will be used to display on LCD
    if (g_figureIndex < FIRST_20_SECOND_DATA_NUM) {
        g_distanceinFirst20Sec[g_figureIndex] = *distanceData;
        if (g_figureIndex == 0) {
            g_timeFirst20Sec[g_figureIndex] = g_sampleInterval;
        } else {
            g_timeFirst20Sec[g_figureIndex] = g_timeFirst20Sec[g_figureIndex - 1] + g_sampleInterval;
        }
        g_figureIndex++;
    }

}

void DisplayPlot()
{
    // Display all recorded data on LCD
    for (int i = 1; i < g_figureIndex; i++) {
        g_LCDController.line(AXIS_START_X + LCD_DISPLAY_X_DATA_FACTOR * (i - 1),
            AXIS_START_Y - LCD_DISPLAY_Y_DATA_FACTOR * int(g_distanceinFirst20Sec[i - 1]),
            AXIS_START_X + LCD_DISPLAY_X_DATA_FACTOR * i,
            AXIS_START_Y - LCD_DISPLAY_Y_DATA_FACTOR * int(g_distanceinFirst20Sec[i]), WHITE);
    }
}

void DisplayDataOnLCD(double data, Timer *displayRefreshTimer)
{   
    displayRefreshTimer->stop();
    EMBED_PROJECT_LOG("displayTimer: %d", displayRefreshTimer->read_ms());
    // only refresh display for every DISPLAY_TIME_INTERVAL_MS millisecond.
    if (displayRefreshTimer->read_ms() > DISPLAY_TIME_INTERVAL_MS) {
        g_LCDController.set_orientation(0);
        g_LCDController.set_font((unsigned char*)Arial24x23);
        g_LCDController.cls();
        g_LCDController.locate(0, DISPLAY_POS_Y - 80);
        g_LCDController.printf("Distance:");
        g_LCDController.locate(DISPLAY_CENTER_X, DISPLAY_POS_Y -50);
        g_LCDController.printf(" %d Meters", (int16_t)data);
        
        DisplayAxis();
        DisplayPlot();

        displayRefreshTimer->reset();
        if (g_outputTimer >= TWENTY_SECOND_MS) {
            g_LCDController.set_font((unsigned char*)Arial12x12);
            g_LCDController.locate(0, DISPLAY_POS_Y + 180);
            g_LCDController.printf("In the first 20 seconds, you have traveled:");
            g_LCDController.set_font((unsigned char*)Arial24x23);
            if (g_first20SecDistance ==  UNSET_FLAG) {
                g_first20SecDistance = (int16_t)data;
            }
            g_LCDController.printf("%d meters", g_first20SecDistance);
        }
    }
    displayRefreshTimer->start();
}

int main() 
{
    // Store the mean value of first CALIBRATION_DATA_LENGTH values, 
    // which used with DATA_NOISE_THRESHOLD to elimate noise
    int32_t caliData = 0; 
    int16_t sensorRawData = 0;
    double distanceData = 0.0;
    int8_t flag;
    Timer displayRefreshTimer;


    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    g_gyroCs = 1;
    g_spi.format(8,3);
    g_spi.frequency(1000000);
    
 
    InitGyroSensor();
    InitDisplay();
    GetCalibrateData(&caliData); // Calibrate the sensor
    g_processDataTimer.start();
    g_sampleInterval = 0;
    g_outputTimer = 0;
    displayRefreshTimer.start();
    g_first20SecDistance = UNSET_FLAG;
    while(1) {
        flag = ReadGyroXData(&sensorRawData);
        if (flag == READ_OK) {
            // Process raw data in dataBuf. After processing, restore the result back into dataBuf
            ProcessData(sensorRawData, caliData, &distanceData);
            DisplayDataOnLCD(distanceData, &displayRefreshTimer);
        }
        wait_us(100000);
    }
}