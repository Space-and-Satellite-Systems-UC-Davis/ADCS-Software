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
} file_tag;

typedef struct {
    vec3 mag;
    vec3 imu;
    vec3 mdm;
} detumbleLOGdata;

typedef struct {
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
    file_tag header;
} LOG_FILE;

/**
 * @brief open the file for logging
 *
 * @param name name of the file
 * @param tag A tag denoting which kind of log file this is
 *
 * @return a LOG_FILE pointer
 */
LOG_FILE *createFile(char *name, file_tag tag);

/**
 * @brief open the file for logging
 *
 * @param name name of the file
 *
 * @return a LOG_FILE pointer
 */
LOG_FILE *openFile(char *name);

/**
 * @brief Close the log file
 *
 * @param file LOG_FILE pointer
 * @param condition The closing condition of the file
 *
 * This function should write out the final row of the log file before closing
 *
 * YYYY-MM-DD-HH-MM-SS, EXIT_LOG_FILE, <Completion / Error Message>
 */
void closeFile(LOG_FILE *file, char *message);

/**
 * Example File Format:
 * Filename: "YYYY-MM-DD_HH-MM-SS_detumble.csv"
 *
 * File Content:
 * detumblingLOG, IMU01, IMU02, MAG01, MAG02\n
 * YYYY-MM-DD_HH-MM-SS, xxx, xxx, xxx \n
 * YYYY-MM-DD_HH-MM-SS, xxx, xxx, xxx \n
 * YYYY-MM-DD_HH-MM-SS, xxx, xxx, xxx \n
 * YYYY-MM-DD-HH-MM-SS, EXIT_LOG_FILE, Completed
 *
 * Essentially we're trying to make csv file to log the data
 */

/**
 * @brief Write the header section of the log file
 *
 * This is to create a header so that the data becomes more readable
 *
 * The first entry would be the
 *
 * @param file LOG_FILE pointer
 *
 * Example: detumblingLOG, IMU01, IMU02, MAG01, MAG02\n
 */
void logHeader(LOG_FILE *file);

/**
 * @brief Append a new entry to the log file
 *
 * @param file LOG_FILE pointer
 * @param record the data to store in the
 *
 * This function should attach a timeStamp at the beginning of every log entry.
 *
 * Example: YYYY-MM-DD_HH-MM-SS, <record>
 */
void logRecord(LOG_FILE *file, void *record);

/**
 * @brief Return all the bytes of a log file
 *
 * @param file LOG_FILE pointer
 *
 * @return Byte buffer storing contents of the file
 */
uint8_t *readHeader(LOG_FILE *file);

/**
 * @brief Return all the bytes of a log file
 *
 * @param file LOG_FILE pointer
 *
 * @return Byte buffer storing contents of the file
 */
uint8_t *readRecord(LOG_FILE *file);

/**
 * @brief List the log files currently in the file system
 *
 * @return string of the list of log files
 */
char *lsRecord();

#endif // VIRTUAL_LOGGERS_H