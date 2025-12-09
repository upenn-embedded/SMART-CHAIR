#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "driver/i2s.h"

// ---------------- I2S PINS ----------------
#define BCLK   11
#define LRCLK  10
#define DIN     7

// ---------------- SD PINS ----------------
#define SD_CS   18
#define SPI_SCK 36
#define SPI_MISO 37
#define SPI_MOSI 35

// UART to ATmega
#define UART_RX_PIN  6
#define UART_TX_PIN  5
#define UART_BAUD    115200

// ---------------- WAV FILE NAME ----------
const char* WAV_FILE = "/music.wav"; 

// ---------------- GLOBALS ----------------
uint16_t g_numChannels = 2; // Store channels here so we don't re-read header

// -----------------------------------------------------------
// I2S CONFIG
// -----------------------------------------------------------
static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 16,
    .dma_buf_len = 1024,
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
};

static const i2s_pin_config_t pin_config = {
    .bck_io_num = BCLK,
    .ws_io_num = LRCLK,
    .data_out_num = DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
};

File wavFile;
bool playing = false;

// Helper to read little-endian 32-bit
static uint32_t read_le_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
// Helper to read little-endian 16-bit
static uint16_t read_le_u16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// -----------------------------------------------------------
// ROBUST WAV PARSER
// "Walks" through chunks to find 'fmt ' and 'data' regardless of location
// -----------------------------------------------------------
bool parseWavHeader(File &f, uint32_t &dataOffset, uint32_t &dataSize)
{
    uint8_t buf[12]; 

    // 1. Read Main Header (12 bytes)
    if (f.read(buf, 12) != 12) return false;

    // Check "RIFF" and "WAVE"
    if (memcmp(buf, "RIFF", 4) != 0) { Serial.println("No RIFF"); return false; }
    if (memcmp(buf + 8, "WAVE", 4) != 0) { Serial.println("No WAVE"); return false; }

    // 2. Iterate chunks
    while (f.available()) 
    {
        uint8_t chunkID[4];
        if (f.read(chunkID, 4) != 4) break; // End of file

        uint8_t sizeBuf[4];
        if (f.read(sizeBuf, 4) != 4) break;
        uint32_t chunkSize = read_le_u32(sizeBuf);

        // --- Handle "fmt " Chunk ---
        if (memcmp(chunkID, "fmt ", 4) == 0) 
        {
            // Read fmt data
            uint8_t fmtData[16];
            if (f.read(fmtData, 16) != 16) return false;
            
            uint16_t audioFormat = read_le_u16(fmtData);
            g_numChannels        = read_le_u16(fmtData + 2);
            uint32_t sampleRate  = read_le_u32(fmtData + 4);
            uint16_t bitsPerSam  = read_le_u16(fmtData + 14);

            Serial.printf("FMT found: Format=%d Ch=%d Rate=%d Bits=%d\n", audioFormat, g_numChannels, sampleRate, bitsPerSam);

            if (audioFormat != 1) { Serial.println("Error: Not PCM"); return false; }
            if (bitsPerSam != 16) { Serial.println("Error: Not 16-bit"); return false; }

            // Apply Sample Rate to I2S
            i2s_set_sample_rates(I2S_NUM_0, sampleRate);

            // Handle extra format bytes if chunkSize > 16
            if (chunkSize > 16) {
                f.seek(f.position() + (chunkSize - 16));
            }
        }
        // --- Handle "data" Chunk ---
        else if (memcmp(chunkID, "data", 4) == 0) 
        {
            Serial.println("DATA chunk found!");
            dataSize = chunkSize;
            dataOffset = f.position(); // Current position is start of audio
            return true; // Success
        }
        // --- Handle Unknown Chunks (LIST, INFO, id3, etc.) ---
        else 
        {
            Serial.print("Skipping chunk: ");
            Serial.write(chunkID, 4);
            Serial.println();
            // Skip this chunk entirely
            f.seek(f.position() + chunkSize);
        }
    }

    Serial.println("Error: 'data' chunk not found");
    return false;
}

// -----------------------------------------------------------
// Play WAV once (blocking)
// -----------------------------------------------------------
void playWavFile(const char* path)
{
    if (!SD.exists(path)) return;
    wavFile = SD.open(path, FILE_READ);
    if (!wavFile) return;

    // (Header parsing skipped for brevity, assume it's done or same as before)
    // ... [Insert parseWavHeader call here] ...
    uint32_t dataOffset = 0;
    uint32_t dataSize = 0;
    parseWavHeader(wavFile, dataOffset, dataSize); 

    const size_t BUF_SAMPLES = 512;        
    uint8_t sdBuf[BUF_SAMPLES * 4];       
    uint8_t i2sBuf[BUF_SAMPLES * 4];      

    // --- PLAYBACK LOOP ---
    while (dataSize > 0) {
        if (Serial1.available()) {
            if (Serial1.read() == 'b') {
                Serial.println("Stopping...");
                playing = false; 
                break; // <--- BREAKS out of the while loop
            }
        }

        // Read and Write Audio
        size_t toRead = (sizeof(sdBuf) > dataSize) ? dataSize : sizeof(sdBuf);
        size_t n = wavFile.read(sdBuf, toRead);
        if (n == 0) break;
        dataSize -= n;

        memcpy(i2sBuf, sdBuf, n); 
        
        size_t written;
        i2s_write(I2S_NUM_0, i2sBuf, n, &written, portMAX_DELAY);
    }

    // --- CLEANUP (Happens after 'break') ---
    wavFile.close();                // 1. Close File
    i2s_zero_dma_buffer(I2S_NUM_0); // 2. Silence Speaker
    Serial.println("Music Stopped.");
}
void setup()
{
    // UART1 for ATmega
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.begin(115200);
    delay(500);
    Serial.println("ESP32: Robust WAV Player");

    // SD init
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    if (!SD.begin(SD_CS)) {
        Serial.println("Error: SD card init failed!");
        while (true) delay(1000);
    }
    Serial.println("SD OK");

    // Setup I2S
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_sample_rates(I2S_NUM_0, 44100); 

    Serial.println("Ready.");
}

void loop()
{
    while (Serial1.available() > 0) {
    char c = Serial1.read();
    Serial.print("ATmega: ");
    Serial.println(c);

    if (c == 'a') {
      if (!playing) {
        playWavFile(WAV_FILE);
        playing = true;
      }
    } else if (c == 'b') {
    if (playing) {
         i2s_zero_dma_buffer(I2S_NUM_0);
         wavFile.close();
        Serial.println("Music STOPPED");
        playing = false;
      }
  }
}
    delay(100);
}
