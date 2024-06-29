#define Stat_Pull_interval_ms 5000
#define LedPin 22
#define NUM_LEDS 120
const int samplingFrequency = 44100;                // The audio sampling frequency. 
const int SAMPLEBLOCK = 1024;                       // size of sample block, only half the size contains useable samples
#define ADC_INPUT ADC1_CHANNEL_0                        //*
int numBands=32;
#define MatrixHeight 16
volatile float FreqBins[32];                            //*
int NoiseTresshold =        2000;                   // this will effect the upper bands most.
#define GAIN_DAMPEN         2                       // Higher values cause auto gain to react more slowly
int oldBarHeights[32];
#define ARRAYSIZE(a)    (sizeof(a)/sizeof(a[0]))
static int BandCutoffTable[32] =
{
  45, 90, 130, 180, 220, 260, 310, 350,
  390, 440, 480, 525, 650, 825, 1000, 1300,
  1600, 2050, 2500, 3000, 4000, 5125, 6250, 9125,
  12000, 13000, 14000, 15000, 16000, 16500, 17000, 17500
};

// -------------------------------------
// -------   Audio Config   ------
// -------------------------------------
// Digital I/O used for soundboard
#define I2S_DOUT      14
#define I2S_BCLK      27
#define I2S_LRC       19
//Audio out on I2S0--> audio library
// Define input buffer length
#define bufferLen 64
#define PotmeterVolumePin 35

// Connections to INMP441 I2S microphone
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
// Use I2S Processor 1 for mic in
#define I2S_PORT I2S_NUM_1

//------- Replace the following! ------
const char ssid[] = "em77e";       // your network SSID (name)
const char password[] = "rtykERS01";  // your network key
#define API_KEY "AIzaSyBcUmygiCcNHyaHzZMnA2sclo" // your Google API key
#define CHANNEL_ID "UCm5wy-G2F9wpDFF3w" // part of the channel url




// no need to change anything below this line.



//****************************************************************
// Return the frequency corresponding to the Nth sample bucket.  
// Skips the first two
// buckets which are overall amplitude and something else.
int BucketFrequency(int iBucket) {
  if (iBucket <= 1)return 0;
  int iOffset = iBucket - 2;
  return iOffset * (samplingFrequency / 2) / (SAMPLEBLOCK / 2);
}

void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = samplingFrequency,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
    .dma_buf_count = 2,//8
    .dma_buf_len = SAMPLEBLOCK,
    .use_apll = false,
        .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void Setup_Wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Connect to the WiFi network
  Serial.print("\nConnecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
 // client.setInsecure();
}
