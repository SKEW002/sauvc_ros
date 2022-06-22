#ifndef PING_SONAR_H
#define PING_SONAR_H

#include "config.h"

static Ping1D ping { Serial1 };

void initialize_ping_sonar();
int get_depth();

#endif
