#include "ESPWiFiSetup.h"
#include <driver/i2s.h>
#include <ArduinoJson.h>
 
const char* Client_ID = "1234";

#define LED_PIN    48      // Onboard RGB pin (adjust if needed)
#define LED_COUNT  1       // Only one RGB LED
//MQTT
const char* mqtt_server = "167.71.237.218";
const int mqtt_port = 1883;
const char* MQTT_TOPIC = "buddy/1234/audio/new";  // /buddy/<client id>/audio/new
 
 
// UDP
WiFiUDP udp;
const char* udpServerIP = "192.168.1.2";
const uint16_t udpPort = 5005;
 
bool lastButtonState = HIGH;  // initial state: unpressed
bool streamActive = false;
 

// Device Identifiers
String deviceKey = "e4b063b90be8";
String serialKey = "123456789012";
String devicePassword = "abcdefghijkl";
String receivedToken = "";
bool tokenReceived = false;
bool mqttInfoReceived = false;
bool mqttTaskDone = false;
 
// I2S Pins
// Mic Pins
#define I2S_MIC_WS   1
#define I2S_MIC_SD   2
#define I2S_MIC_SCK  42
 
// Speaker Pins
#define I2S_SPK_WS   39
#define I2S_SPK_SD   41
#define I2S_SPK_SCK  40
 
 
#define SAMPLE_RATE  16000
#define SAMPLE_COUNT 8000     // 1 second of audio
#define UDP_CHUNK_SIZE 1024
 
#define BUTTON_PIN 4
 
WiFiClient wifiClient;
PubSubClient client(wifiClient);
 
bool playRequested = false;
int16_t audioBuffer[SAMPLE_COUNT];     // 32KB buffer
int16_t playbackBuffer[SAMPLE_COUNT];  // for playback
 
void sendAuth() {
  String topic = "/user/folotoy/" + deviceKey + "/thing/command/auth";
  String payload = "{\"serialKey\":\"" + serialKey + "\",\"password\":\"" + devicePassword + "\"}";
  client.publish(topic.c_str(), payload.c_str());
  Serial.println("[TOY] Sent serialKey & password");
}

void sendTokenAck() {
  String topic = "/user/folotoy/" + deviceKey + "/thing/command/tokenAck";
  String payload = "{\"token\":\"" + receivedToken + "\"}";
  client.publish(topic.c_str(), payload.c_str());
  Serial.println("[TOY] Sent token ACK");
}

void sendMqttInfoAck() {
  String topic = "/user/folotoy/" + deviceKey + "/thing/command/mqttinfoAck";
  client.publish(topic.c_str(), "{\"ack\":1}");
  Serial.println("[TOY] Sent MQTT info ACK");
  mqttTaskDone = true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  Serial.println("[TOY] Received: " + msg);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) return;

  String topicStr = String(topic);
  if (topicStr.endsWith("/token")) {
    receivedToken = doc["token"].as<String>();
    tokenReceived = true;
    sendTokenAck();
  } 
  else if (topicStr.endsWith("/mqttinfo")) {
    String broker = doc["broker"].as<String>();
    int port = doc["port"].as<int>();
    mqttInfoReceived = true;
    Serial.println("[TOY] MQTT Info Received:");
    Serial.println("Broker: " + broker);
    Serial.println("Port: " + String(port));
    sendMqttInfoAck();
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(deviceKey.c_str())) {
      // Subscribe to expected server topics
      client.subscribe(("/user/folotoy/" + deviceKey + "/thing/command/token").c_str());
      client.subscribe(("/user/folotoy/" + deviceKey + "/thing/command/mqttinfo").c_str());
      Serial.println("[TOY] MQTT connected and subscribed");
    } else {
      delay(2000);
    }
  }
}
 
 
void setupMic() {
  i2s_config_t micConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE, // Updated to 16000 Hz
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
};
 
  i2s_pin_config_t micPins = {
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };
 
  i2s_driver_install(I2S_NUM_0, &micConfig, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &micPins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}
 
void setupSpeaker() {
  i2s_config_t spkConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
 
  i2s_pin_config_t spkPins = {
    .bck_io_num = I2S_SPK_SCK,
    .ws_io_num = I2S_SPK_WS,
    .data_out_num = I2S_SPK_SD,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
 
  i2s_driver_install(I2S_NUM_1, &spkConfig, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &spkPins);
  i2s_zero_dma_buffer(I2S_NUM_1);
}
 
// Record exactly 1 second of audio
void recordOneSecondAudio() {
  Serial.println("Recording 1 second...");
  size_t bytesReadTotal = 0;
  while (bytesReadTotal < sizeof(audioBuffer)) {
    size_t bytesRead = 0;
    i2s_read(I2S_NUM_0,
             (uint8_t*)audioBuffer + bytesReadTotal,
             sizeof(audioBuffer) - bytesReadTotal,
             &bytesRead,
             portMAX_DELAY);
    bytesReadTotal += bytesRead;
  }
  memcpy(playbackBuffer, audioBuffer, sizeof(audioBuffer));
  Serial.println("Recorded 1 second.");
}
 

// Send buffer over UDP in chunks
void sendAudioUDP() {
  Serial.println("Sending audio in 1024-byte chunks over UDP...");
 
  // Calculate how many 1024-byte chunks we need to send
  const size_t CHUNK_SIZE = 1024;
  const size_t audioSize = sizeof(audioBuffer);
  const size_t totalChunks = (audioSize + CHUNK_SIZE - 1) / CHUNK_SIZE; // Ceiling division
 
  // Create buffer for each packet
  uint8_t packet[CHUNK_SIZE];
 
  // Send audio data in chunks
  size_t totalSent = 0;
  for (size_t i = 0; i < totalChunks; i++) {
      // Calculate size of current chunk (may be smaller for last chunk)
      size_t remainingBytes = audioSize - totalSent;
      size_t currentChunkSize = (remainingBytes > CHUNK_SIZE) ? CHUNK_SIZE : remainingBytes;
 
      // Copy current chunk of audio data to packet buffer
      memcpy(packet, ((uint8_t*)audioBuffer) + totalSent, currentChunkSize);
 
      // Debug logs to verify chunk size
      Serial.print("Sending audio chunk of size: ");
      Serial.println(currentChunkSize);
 
      // Send the packet
      udp.beginPacket(udpServerIP, udpPort);
      udp.write(packet, currentChunkSize);
      udp.endPacket();
 
      totalSent += currentChunkSize;
 
      // Optional: Small delay to avoid network congestion
      delay(1);
  }
 
  Serial.print("Sent audio data in ");
  Serial.print(totalChunks);
  Serial.println(" chunks.");
}
 
void sendStopSignal() {
  Serial.println("Sending STOP signal...");
 
  // Create a 4-byte packet for the STOP signal
  uint8_t packet[4];
  memcpy(packet, "STOP", 4);
 
  udp.beginPacket(udpServerIP, udpPort);
  udp.write(packet, 4);
  udp.endPacket();
 
  Serial.println("STOP signal sent.");
}
 
 
// Play back 1 second of recorded audio
void playAudioChunk() {
  size_t bytesWritten;
  i2s_write(I2S_NUM_1, (void*)playbackBuffer, sizeof(playbackBuffer), &bytesWritten, portMAX_DELAY);
  Serial.println("Playback done.");
}
 
// SET_LOOP_TASK_STACK_SIZE(16 * 1024); // Sets stack size to 16KB
 
void setup() {
  Serial.begin(115200);
  wifisetup();
  dnsloop();
  // connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  sendAuth();  // Step 1: Send serialKey & password

  while (!mqttTaskDone) {
    client.loop();
    delay(10);
  }
  
  udp.begin(12345);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  setupMic();
  setupSpeaker();
}
 
unsigned long lastTime = 0;
 
void loop() {


  handleResetButton();  // Check for button press to reset Wi-Fi credentials
 
  bool currentButtonState = digitalRead(BUTTON_PIN);
 
  // Detect rising edge (button released)
  if (lastButtonState == LOW && currentButtonState == HIGH) {
    Serial.println("Button Released!");
    streamActive = false;
    sendStopSignal();
  }
 
  // Detect falling edge (button pressed)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    Serial.println("Button Pressed!");
    streamActive = true;
  }
 
  lastButtonState = currentButtonState;
 
  // If streaming is active, record & send audio continuously
  if (streamActive) {
    size_t bytesRead = 0;
    uint8_t audioChunk[UDP_CHUNK_SIZE]; // Temporary buffer for audio chunk
 
    // Read audio data from the microphone
    i2s_read(I2S_NUM_0, audioChunk, sizeof(audioChunk), &bytesRead, portMAX_DELAY);
 
    if (bytesRead > 0) {
      // Send the audio chunk over UDP
      udp.beginPacket(udpServerIP, udpPort);
      udp.write(audioChunk, bytesRead);
      udp.endPacket();
 
      Serial.print("Sent audio chunk of size: ");
      Serial.println(bytesRead);
    }
  }
 
  // Handle play request from MQTT
  if (playRequested) {
    playAudioChunk();
    playRequested = false;
  }
}









