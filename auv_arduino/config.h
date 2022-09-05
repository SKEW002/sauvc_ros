# ifndef SAUVC2022_CONFIG_H
# define SAUVC2022_CONFIG_H

#include "ping1d.h"
#include <stdio.h>
/**
 * Motor to Arduino Pin Connection.
 */
# define MOTOR_NO_1_PIN int(9)
# define MOTOR_NO_2_PIN int(11)
# define MOTOR_NO_3_PIN int(6)
# define MOTOR_NO_4_PIN int(8)
# define MOTOR_NO_5_PIN int(13)
# define MOTOR_NO_6_PIN int(12)
# define MOTOR_NO_7_PIN int(7)
# define MOTOR_NO_8_PIN int(10)

/**
 * E stop switch to Arduino Pin Connection.
 */
# define E_STOP_PIN int(2)

/**
 * ESC Input Value Safety Limit
 */
# define MAX_ESC_INPUT int(1900)
# define MIN_ESC_INPUT int(1100)

/**
 * ESC Input Value for Stop Signal
 */
# define STOP_SIGNAL int(1500)

/**
 * Blue Robotics Ping Sonar
 */
static const uint8_t arduinoRxPin = 19; //Serial1 rx
static const uint8_t arduinoTxPin = 18; //Serial1 tx

static const uint8_t ledPin = 13;


/**
 * Blue Robotics Bar02
 */
# define DEPTH_SENSOR_MODEL MS5837::MS5837_02BA
# define FLUID_DENSITY 997 // kg/m^3 (freshwater, 1029 for seawater)
# define OPERATING_DEPTH float()
# define DEPTH_TOLERANCE float(3.25)



/**
 * ESC Input Value of motors for each motion. (for reference only)

# define MOTOR_AND_ESC_INPUT_FOR_FORWARD {MOTOR_NO_1_PIN, 1700}, \
                                         {MOTOR_NO_2_PIN, 1700}, \
                                         {MOTOR_NO_3_PIN, 1700}, \
                                         {MOTOR_NO_4_PIN, 1700}, \
                                         {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_BACKWARD {MOTOR_NO_1_PIN, 1400}, \
                                          {MOTOR_NO_2_PIN, 1400}, \
                                          {MOTOR_NO_3_PIN, 1400}, \
                                          {MOTOR_NO_4_PIN, 1400}, \
                                          {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_SUBMERGE {MOTOR_NO_1_PIN, 1700}, \
                                          {MOTOR_NO_2_PIN, 1700}, \
                                          {MOTOR_NO_3_PIN, 1700}, \
                                          {MOTOR_NO_4_PIN, 1700}, \
                                          {MOTOR_NO_5_PIN, 1400}, \
                                          {MOTOR_NO_6_PIN, 1600}, \
                                          {MOTOR_NO_7_PIN, 1600}, \
                                          {MOTOR_NO_8_PIN, 1400}
# define MOTOR_AND_ESC_INPUT_FOR_SURFACE {MOTOR_NO_1_PIN, 1700}, \
                                         {MOTOR_NO_2_PIN, 1700}, \
                                         {MOTOR_NO_3_PIN, 1700}, \
                                         {MOTOR_NO_4_PIN, 1700}, \
                                         {MOTOR_NO_5_PIN, 1600}, \
                                         {MOTOR_NO_6_PIN, 1400}, \
                                         {MOTOR_NO_7_PIN, 1400}, \
                                         {MOTOR_NO_8_PIN, 1600}
# define MOTOR_AND_ESC_INPUT_FOR_ROTATE_LEFT {MOTOR_NO_1_PIN, 1600}, \
                                             {MOTOR_NO_2_PIN, 1400}, \
                                             {MOTOR_NO_3_PIN, 1600}, \
                                             {MOTOR_NO_4_PIN, 1400}, \
                                             {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                             {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                             {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                             {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_ROTATE_RIGHT {MOTOR_NO_1_PIN, 1400}, \
                                              {MOTOR_NO_2_PIN, 1600}, \
                                              {MOTOR_NO_3_PIN, 1400}, \
                                              {MOTOR_NO_4_PIN, 1600}, \
                                              {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_TRANSLATE_LEFT {MOTOR_NO_1_PIN, 1600}, \
                                                {MOTOR_NO_2_PIN, 1400}, \
                                                {MOTOR_NO_3_PIN, 1400}, \
                                                {MOTOR_NO_4_PIN, 1600}, \
                                                {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_TRANSLATE_RIGHT {MOTOR_NO_1_PIN, 1400}, \
                                                 {MOTOR_NO_2_PIN, 1600}, \
                                                 {MOTOR_NO_3_PIN, 1600}, \
                                                 {MOTOR_NO_4_PIN, 1400}, \
                                                 {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_ROLL_LEFT {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                           {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                           {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                           {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                           {MOTOR_NO_5_PIN, 1400}, \
                                           {MOTOR_NO_6_PIN, 1400}, \
                                           {MOTOR_NO_7_PIN, 1600}, \
                                           {MOTOR_NO_8_PIN, 1600}
# define MOTOR_AND_ESC_INPUT_FOR_ROLL_RIGHT {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                            {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                            {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                            {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                            {MOTOR_NO_5_PIN, 1600}, \
                                            {MOTOR_NO_6_PIN, 1600}, \
                                            {MOTOR_NO_7_PIN, 1400}, \
                                            {MOTOR_NO_8_PIN, 1400}
# define MOTOR_AND_ESC_INPUT_FOR_PITCH_FORWARD {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {MOTOR_NO_5_PIN, 1400}, \
                                               {MOTOR_NO_6_PIN, 1600}, \
                                               {MOTOR_NO_7_PIN, 1400}, \
                                               {MOTOR_NO_8_PIN, 1600}
# define MOTOR_AND_ESC_INPUT_FOR_PITCH_BACKWARD {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_5_PIN, 1600}, \
                                                {MOTOR_NO_6_PIN, 1400}, \
                                                {MOTOR_NO_7_PIN, 1600}, \
                                                {MOTOR_NO_8_PIN, 1400}

*/


# endif
