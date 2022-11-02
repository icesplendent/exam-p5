#include "accelerometer.h"
#include "gyro.h"
#include "mbed.h"

const int16_t waveformLength = 128;
const int16_t lookUpTableDelay = 10;

AnalogOut Aout(PA_4);
InterruptIn btnRecord(BUTTON1);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
EventQueue queue_note(32 * EVENTS_EVENT_SIZE);
Thread t;
Thread t_note;
Accelerometer acc;
Gyro gyro;
double Accel[3] = {0};
double Gyro[3] = {0};
double accAngleX = 0;
double accAngleY = 0;
double elapsedTime = 0;
double roll, pitch, yaw;
double gyroAngleX = 0;
double gyroAngleY = 0;
int counter = 0;
int idR[32] = {0};
int indexR = 0;

int idNote[32] = {0};
int indexNote = 0;
int index_pitch = 0;
const float C_4 = 261.63;
const float D_4 = 293.66;
const float E_4 = 329.63;
const float F_4 = 349.23;
const float G_4 = 392.00;
const float A_4 = 440.00;
const float B_4 = 493.88;
const float C_5 = 523.25;
const int length = 8;
float song[length] = {C_4, D_4, E_4, F_4, G_4, A_4, B_4, C_5};

// note length in second
int8_t noteLength[length] = {1, 1, 1, 1, 1, 1, 1, 1};

float volume[length] = {1, 1, 1, 1, 1, 1, 1, 1};

float waveform[waveformLength] = {  // 128 samples of a sine waveform
    0.500, 0.525, 0.549, 0.574, 0.598, 0.622, 0.646, 0.670, 0.693, 0.715, 0.737,
    0.759, 0.780, 0.800, 0.819, 0.838, 0.856, 0.873, 0.889, 0.904, 0.918, 0.931,
    0.943, 0.954, 0.964, 0.972, 0.980, 0.986, 0.991, 0.995, 0.998, 1.000, 1.000,
    0.999, 0.997, 0.994, 0.989, 0.983, 0.976, 0.968, 0.959, 0.949, 0.937, 0.925,
    0.911, 0.896, 0.881, 0.864, 0.847, 0.829, 0.810, 0.790, 0.769, 0.748, 0.726,
    0.704, 0.681, 0.658, 0.634, 0.610, 0.586, 0.562, 0.537, 0.512, 0.488, 0.463,
    0.438, 0.414, 0.390, 0.366, 0.342, 0.319, 0.296, 0.274, 0.252, 0.231, 0.210,
    0.190, 0.171, 0.153, 0.136, 0.119, 0.104, 0.089, 0.075, 0.063, 0.051, 0.041,
    0.032, 0.024, 0.017, 0.011, 0.006, 0.003, 0.001, 0.000, 0.000, 0.002, 0.005,
    0.009, 0.014, 0.020, 0.028, 0.036, 0.046, 0.057, 0.069, 0.082, 0.096, 0.111,
    0.127, 0.144, 0.162, 0.181, 0.200, 0.220, 0.241, 0.263, 0.285, 0.307, 0.330,
    0.354, 0.378, 0.402, 0.426, 0.451, 0.475, 0.500};

// duration in s
void playNote(int freq, int duration, float vol) {
    int j = waveformLength;
    int waitTime = (1000000 / waveformLength / freq - lookUpTableDelay) << 0;
    int i = duration * freq;  // play i iterations of waveform samples
    printf("Play %d for %d at %f\n", freq, duration, vol);
    while (i--) {
        j = waveformLength;
        while (j--) {
            Aout = waveform[j] * vol;  // scale with volume
            wait_us(waitTime);
        }
    }
}

void record(void) {
    acc.GetAcceleromterSensor(Accel);
    acc.GetAcceleromterCalibratedData(Accel);

    // printf("Calibrated ACC= %f, %f, %f\n", Accel[0], Accel[1], Accel[2]);

    // Calculating Roll and Pitch from the accelerometer data
    accAngleX =
        (atan(Accel[1] / sqrt(Accel[0] * Accel[1] + Accel[2] * Accel[2])) *
         180 / SENSOR_PI_DOUBLE);
    accAngleY =
        (atan(-1 * Accel[1] / sqrt(Accel[1] * Accel[1] + Accel[2] * Accel[2])) *
         180 / SENSOR_PI_DOUBLE);

    gyro.GetGyroSensor(Gyro);
    gyro.GetGyroCalibratedData(Gyro);

    // printf("Calibrated Gyro= %f, %f, %f\n", Gyro[0], Gyro[1], Gyro[2]);
    elapsedTime = 0.01;  // 10ms by thread sleep time
    // Currently the raw values are in degrees per seconds, deg/s, so we need to
    // multiply by sendonds (s) to get the angle in degrees
    gyroAngleX = gyroAngleX + Gyro[0] * elapsedTime;  // deg/s * s = deg
    gyroAngleY = gyroAngleY + Gyro[1] * elapsedTime;
    yaw = yaw + Gyro[2] * elapsedTime;
    // Complementary filter - combine acceleromter and gyro angle values
    // roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    // pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    // Use Acc data only
    roll = accAngleX;
    pitch = accAngleY;
    printf("%f/%f/%f\n", roll, pitch, yaw);
    if (pitch > 10 || roll > 10) {
        index_pitch++;
        if (index_pitch > 7) index_pitch = 7;  // saturate at C_5
    } else if (pitch < -10 || roll < 10) {
        index_pitch--;
        if (index_pitch < 0) index_pitch = 0;  // saturate at C_4
    }
    // printf("index_pitch=%d\n", index_pitch);
    // Play note
    idNote[indexNote] =
        queue_note.call(playNote, song[index_pitch], noteLength[index_pitch],
                        volume[index_pitch]);
    indexNote = indexNote % 32;

    // ThisThread::sleep_for(10ms);
}

void startRecord(void) {
    // printf("---start---\n");
    idR[indexR++] = queue.call_every(1000ms, record);
    indexR = indexR % 32;

    index_pitch = 0;  // starting with C_4
}

void stopRecord(void) {
    // printf("---stop---\n");
    for (auto &i : idR) queue.cancel(i);

    for (auto &i : idNote) queue_note.cancel(i);
}

int main() {
    // printf("Init accelerometer and Gyro\n");
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    t_note.start(callback(&queue_note, &EventQueue::dispatch_forever));
    btnRecord.fall(queue.event(startRecord));
    btnRecord.rise(queue.event(stopRecord));
}
