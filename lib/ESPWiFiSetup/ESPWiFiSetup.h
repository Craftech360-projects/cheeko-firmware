#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include "nvs_flash.h"
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>

const byte DNS_PORT = 53;
DNSServer dnsServer;
WebServer server(80);
Preferences prefs;
#define RESET_BUTTON_PIN 15  // Change as needed
bool shouldStartAP = false;
String ssidList;

#define LED_PIN    48      // Onboard RGB pin (adjust if needed)
#define LED_COUNT  1       // Only one RGB LED

Adafruit_NeoPixel rgb(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void wifisetup();
void handleResetButton();
void dnsloop();


void scanNetworks() {
    int n = WiFi.scanNetworks();
    ssidList = "<select name='ssid'>";
    for (int i = 0; i < n; i++) {
      ssidList += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i) + "</option>";
    }
    ssidList += "</select>";
  }
  
  // Captive portal root page (Wi-Fi setup form)
  void handleRoot() {
    scanNetworks();
    String page = R"rawliteral(
      <html><head><title>ESP32 WiFi Setup</title></head><body>
      <h2>Configure WiFi</h2>
      <form action="/connect" method="POST">
        <label>SSID:</label> %SSID_LIST%<br>
        <label>Password:</label><input type="password" name="password"><br>
        <input type="submit" value="Connect">
      </form>
      <br><a href="/forget">Forget Wi-Fi</a>
      </body></html>
    )rawliteral";
    page.replace("%SSID_LIST%", ssidList);
    server.send(200, "text/html", page);
  }
  
  // Handle Wi-Fi connection attempt
  void handleConnect() {
    String ssid = server.arg("ssid");
    String password = server.arg("password");
  
    WiFi.begin(ssid.c_str(), password.c_str());
    server.send(200, "text/html", "<h2>Connecting...</h2><meta http-equiv='refresh' content='5; URL=/'/>");
  
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
      delay(100);
    }
  
    if (WiFi.status() == WL_CONNECTED) {
      // Save credentials if connected
      prefs.putString("ssid", ssid);
      prefs.putString("pass", password);
  
      String page = "<html><body><h2>✅ Connected to " + ssid + "</h2>";
      page += "<p>IP Address: " + WiFi.localIP().toString() + "</p>";
      page += "<br><a href='/forget'>❌ Forget Wi-Fi</a>";
      page += "</body></html>";
      server.send(200, "text/html", page);
  
      delay(3000); // Allow user to see the message
      WiFi.mode(WIFI_STA); // Switch to Station Mode
      WiFi.softAPdisconnect(true); // Disconnect AP mode
      Serial.println("✅ Connected and AP turned off.");
      shouldStartAP = true;

    } else {
      server.send(200, "text/html", "<h2>Failed to connect. <a href='/'>Try again</a></h2>");
    }
  }
  
  // Forget saved Wi-Fi credentials
  void handleForget() {
    prefs.remove("ssid");
    prefs.remove("pass");
    server.send(200, "text/html", "<h2>WiFi credentials forgotten. Rebooting...</h2>");
    delay(2000);
    ESP.restart();
  }
  
  // Switch to AP mode to allow new Wi-Fi setup
  void switchToAPMode() {
    // Disconnect current Wi-Fi connection and switch to AP mode
    rgb.setPixelColor(0, rgb.Color(255, 255, 0));  // Yellow
    rgb.show();
    WiFi.disconnect();
    WiFi.mode(WIFI_AP); // Switch to AP mode
    WiFi.softAP("ESP32_Setup");
    delay(100);
  
    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  
    dnsServer.start(DNS_PORT, "*", apIP); // Start DNS for captive portal
  
    server.on("/", handleRoot);
    server.on("/connect", HTTP_POST, handleConnect);
    server.on("/forget", handleForget);
    server.onNotFound([]() {
      server.sendHeader("Location", "/");
      server.send(302, "text/plain", "Redirecting...");
    });
  
    server.begin();
    Serial.println("📡 Captive portal started at http://192.168.4.1");
  }
  
  // Handle button press to trigger switch to AP mode and reset credentials
  void handleResetButton() {
    if (digitalRead(RESET_BUTTON_PIN) == LOW) {
      Serial.println("❌ Reset button pressed. Clearing Wi-Fi credentials.");
      
      // Erase stored Wi-Fi credentials
      prefs.remove("ssid");
      prefs.remove("pass");
  
      // Switch to AP mode and start new Wi-Fi setup process
      switchToAPMode();
    }
  }
  
  void wifisetup() {
    // Serial.begin(115200);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);  // Assuming a reset button on pin 0
    prefs.begin("wifiCreds", false);  // Open Preferences storage for Wi-Fi credentials
    rgb.begin();      // Initialize the NeoPixel library.
    rgb.show();       // Turn off all pixels
    String savedSSID = prefs.getString("ssid", "");
    String savedPASS = prefs.getString("pass", "");
  
    if (savedSSID != "") {
      WiFi.mode(WIFI_STA);
      WiFi.begin(savedSSID.c_str(), savedPASS.c_str());
  
      Serial.print("Connecting to saved Wi-Fi: ");
      Serial.println(savedSSID);
  
      unsigned long startAttempt = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
        delay(100);
      }
  
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ Reconnected using saved credentials.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        return; // Don't start AP mode if connected
      }
    }
  
    // If no saved credentials or failed to connect, start AP mode
    Serial.println("❌ No saved Wi-Fi or failed to connect. Starting AP mode...");
    switchToAPMode();
  }
  
  void dnsloop() {
    while(shouldStartAP == false)
    {
      dnsServer.processNextRequest();
      server.handleClient();
    }
    return;
  }