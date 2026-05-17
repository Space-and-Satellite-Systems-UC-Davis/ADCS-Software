/**
 * @file virtual_loggers.h
 *
 * @brief ADCS Software's interface to Loggers
 *
 *  This file contain function headers for every function
 *  ADCS needs to interface with
 *
 * @author Li, Chun Ho (lchli@ucdavis.edu)
 * @date 05/14/2026
 */

#ifndef VIRTUAL_LOGGERS_H
#define VIRTUAL_LOGGERS_H

#include "adcs_math/vector.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
} TIMESTAMP;

/**
 * @brief Tag denoting which kind of log file
 *
 * @param detumblingLOG
 * @param detumblingSTART TIME, IMU, MAG, TEMP
 *
 */
typedef enum {
    detumbleLOG = 0,   // IMU, MAG,
    detumbleSTART = 1, // TIME, IMU, MAG, TEMP
} log_record_tag;

typedef struct {
    TIMESTAMP time;
    vec3 mag;
    vec3 imu;
    vec3 mdm;
} detumbleLOGdata;

typedef struct {
    TIMESTAMP time;
    vec3 imu;
    vec3 mag;
    double temp;
} detumbleSTARTdata;

/**
 * @brief Create and open the file for logging
 *
 * @param fd file descriptor
 * @param position positsion of file pointer
 * @param row_size the size of each row (uniform across the file)
 * @param header A tag denoting which kind of log file
 */
typedef struct {
    int fd;
    int position;
    size_t row_size;
    log_record_tag header;
} LOG_FILE;

/**
 * @brief open the file for logging
 *
 * @param name name of the file
 * @param tag A tag denoting which kind of log file this is
 *
 * @return a custom log file pointer
 */
LOG_FILE *createFile(char *name, log_record_tag tag);

/**
 * @brief open the file for logging
 *
 * @param name name of the file
 *
 * @return a custom log file pointer
 */
LOG_FILE *openFile(char *name);

/**
 * @brief Close the log file
 *
 * @param file log file pointer
 */
void closeFile(LOG_FILE *file);

/*
 * Example File Format:
 * Filename: "YYYY-MM-DD_HH-MM-SS_detumble.csv"
 * tag: detumblingLOG
 *
 * File Content:
 * TIMESTAMP, IMU01, IMU02, MAG01
 * xxx, xxx, xxx, xxx
 * xxx, xxx, xxx, xxx
 *
 * Essentially we're trying to make csv file to log the data
 */

/**
 * @brief Write the header section of the log file
 *
 * This is to create a header so that the data becomes more readable
 *
 * @param file file pointer
 *
 * @return a custom log file pointer
 */
int logHeader(LOG_FILE *file);

/**
 * @brief Append a new entry to the log file
 *
 * @param name name of the file
 * @param tag A tag denoting which kind of log file this is
 *
 * @return a custom log file pointer
 */
int logRecord(LOG_FILE *file, void *record);

#endif // VIRTUAL_LOGGERS_H