#pragma once
#include <LittleFS.h>

inline File logFile;
inline bool logging = false;
inline char current_path[32];
inline const uint32_t MAX_LOG_BYTES = 1024 * 1024;

inline void startLogging(const char *log_file_name) {
    snprintf(current_path, sizeof(current_path), "/%s.bin", log_file_name);
    LittleFS.remove(current_path);
    logFile = LittleFS.open(current_path, "w");
    logging = true;
}

template<typename T>
void writeSample(const T &sample) {
    if (!logging) return;
    if (logFile.position() >= MAX_LOG_BYTES) {
        logFile.close();
        logFile = LittleFS.open(current_path, "w");
    }
    logFile.write((uint8_t*)&sample, sizeof(T));
}

inline void stopLogging() {
    logFile.close();
    logging = false;
}

inline void dumpLog() {
    File f = LittleFS.open(current_path, "r");
    if (!f) return;
    Serial.write(0xAA); Serial.write(0xBB);
    uint32_t size = f.size();
    Serial.write((uint8_t*)&size, 4);
    while (f.available()) {
        Serial.write(f.read());
    }
    Serial.write(0xCC); Serial.write(0xDD);
    f.close();
}

inline bool isLogging() {
    return logging;
}
