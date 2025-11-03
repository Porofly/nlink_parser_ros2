#ifndef INITSERIAL_H
#define INITSERIAL_H
#include <serial/serial.h>
#include <string>

bool initSerial(serial::Serial& serial, const std::string& param_file_path);

#endif
