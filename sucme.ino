// SUCME
#include "painlessMesh.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <SoftwareSerial.h>
#include <ezButton.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define HAS_IMU
#define HAS_GPS

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define BUTTON_PIN1 19
#define BUTTON_PIN2 18
#define BUTTON_PIN3 5
#define BUTTON_PIN4 4
#define DEBOUNCE_TIME 20
#define MESH_PREFIX "RNTMESH"
#define MESH_PASSWORD "MESHpassword"
#define MESH_PORT 5555

int nodeNumber = 1;
Scheduler userScheduler;
painlessMesh mesh;

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float declinationAngle = 0.05;
ezButton button1(BUTTON_PIN1), button2(BUTTON_PIN2), button3(BUTTON_PIN3), button4(BUTTON_PIN4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

char *defaultMenu[] = {"ALERT", "Send ATAK CoT", "Settings", "Custom Message"};
String contact, distance, order;
char *contacts[] = {"Inf", "Vec", "Obj", "FS", "LP/OP", "Comm", "FOB", "Friend", "Unkn"};
char *distances[] = {"<25m", "25m", "50m", "100m", "150m", "200m", "300m", "400m", "500m"};
char *orders[] = {"ENGAGE", "Observe", "Retreat", "Follow", "Mark", "Regroup", "Dig-in", "Spread out", "Hold"};
String lastMenu[] = {defaultMenu[0], defaultMenu[1], defaultMenu[2], defaultMenu[3]};

float latitude, longitude;
float headingDegrees;
String lastBearing;
String bearing;
bool displayNeedsUpdate = true, newMsg = false;
int cdo = 0, menu = 0;
unsigned long lastMsgTime = 0;
String incomingMsg;

char currentCallsign[5] = "____"; // Current callsign
char newCallsign[5] = "____";    // Callsign being constructed
int newCallsignIndex = 0;        // Current index being edited
char charList[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
int charListSize = sizeof(charList) - 1; // Exclude null terminator
int charCursor = 0;                      // Cursor position in the charList
int settingsOption = 0;          // Tracks selected option in the Settings menu

void markDisplayForUpdate() {
    //Serial.println("Marking display for update");
    displayNeedsUpdate = true;
}

void updateDisplay(String one, String two, String three, String four) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    if (cdo == 99) { // Settings menu
        display.setCursor(0, 0);
        display.print("1. " + one);
        display.setCursor(0, 10);
        display.print("2. " + two);
        display.setCursor(0, 20);
        display.print("3. " + three);
        display.setCursor(0, 30);
        display.print("4. " + four);
    } else if (settingsOption == 1) { // Callsign editor
        // Display current and new callsign
        display.setCursor(0, 0);
        display.print("Current: ");
        display.print(currentCallsign);
        display.setCursor(0, 10);
        display.print("New: ");
        display.print(newCallsign);

        // Display unified character list
        display.setCursor(0, 20);
        for (int i = 0; i < charListSize; i++) {
            if (i == charCursor) display.print("[");
            display.print(charList[i]);
            if (i == charCursor) display.print("]");
        }
    } else {
        // Default menu
        display.setCursor(0, 0);
        display.print("1. " + one);
        display.setCursor(110, 0);
        display.print(int(headingDegrees));
        display.setCursor(0, 10);
        display.print("2. " + two);
        display.setCursor(116, 10);
        display.print(bearing);
        display.setCursor(0, 20);
        display.print("3. " + three);
        display.setCursor(0, 30);
        display.print("4. " + four);
        display.setCursor(0, 56);
        display.println(incomingMsg);
    }
 
    display.display();
    displayNeedsUpdate = false;
}

void updateDisplayIfNeeded() {
    if (displayNeedsUpdate) {
        //Serial.println("Updating display...");
        updateDisplay(lastMenu[0], lastMenu[1], lastMenu[2], lastMenu[3]);
        displayNeedsUpdate = false; // Reset after updating
    }
}

void handleSettingsNavigation(int buttonNumber) {
    if (buttonNumber == 4) { // Back
        settingsOption = 0;
        cdo = 0;
        menu = 0;
        lastMenu[0] = defaultMenu[0];
        lastMenu[1] = defaultMenu[1];
        lastMenu[2] = defaultMenu[2];
        lastMenu[3] = defaultMenu[3];
        markDisplayForUpdate();
    } else if (buttonNumber == 1) { // Callsign
        settingsOption = 1; // Indicate we are in the callsign menu
        cdo = 98;           // Unique cdo value for callsign editing
        newCallsignIndex = 0;
        memset(newCallsign, '_', sizeof(newCallsign) - 1); // Reset new callsign
        newCallsign[4] = '\0';
        markDisplayForUpdate();
    }
}


void handleCallsignEditing(int buttonNumber) {
    if (buttonNumber == 1) { // Move cursor forward
        charCursor = (charCursor + 1) % charListSize;
        markDisplayForUpdate();
    } else if (buttonNumber == 2) { // Move cursor backward
        charCursor = (charCursor - 1 + charListSize) % charListSize; // Wrap around
        markDisplayForUpdate();
    } else if (buttonNumber == 3) { // Select character
        if (newCallsignIndex < 4) {
            newCallsign[newCallsignIndex++] = charList[charCursor];
            if (newCallsignIndex == 4) { // Callsign complete
                strcpy(currentCallsign, newCallsign);
                handleSettingsNavigation(4); // Go back to the main menu
            }
        }
        markDisplayForUpdate();
    } else if (buttonNumber == 4) { // Back to settings menu
        handleSettingsNavigation(4);
    }
}

void setBearing(float angle) {
    if (angle > 337 || angle < 23) bearing = "N";
    else if (angle > 22 && angle < 68) bearing = "NE";
    else if (angle > 67 && angle < 113) bearing = "E";
    else if (angle > 112 && angle < 158) bearing = "SE";
    else if (angle > 157 && angle < 203) bearing = "S";
    else if (angle > 202 && angle < 248) bearing = "SW";
    else if (angle > 247 && angle < 293) bearing = "W";
    else if (angle > 292 && angle < 338) bearing = "NW";
    
    if (lastBearing != bearing) {
      markDisplayForUpdate();
      lastBearing = bearing;
    }
}

String constructMessage() {
    String lat = (latitude != 0) ? String(latitude) : "N/A";
    String lon = (longitude != 0) ? String(longitude) : "N/A";
    Serial.printf("%s %s from %.6f %.6f %s\n", contact.c_str(), distance.c_str(), latitude, longitude, order.c_str());
    return String(currentCallsign) + ": " + contact + " " + distance + " " + order;
}

void sendMessage() {
    if (newMsg) {
        mesh.sendBroadcast(constructMessage());
        newMsg = false;
    }
}

Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, &sendMessage);

void updateHeading() {
    sensors_event_t event; 
    if (!mag.getEvent(&event)) {
        Serial.println("Failed to get compass event");
        return; // Exit the function if the event retrieval fails
    }
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = 0.22;
    heading += declinationAngle;
    if(heading < 0)
    heading += 2*PI;
    
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
     
    // Convert radians to degrees for readability.
    headingDegrees = heading * 180/M_PI; 
    setBearing(headingDegrees);
    
    Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
}

void updateGPS() {
    //unsigned long start = millis();
    while (Serial2.available() > 0) {
        if (gps.encode(Serial2.read())) {
            if (gps.location.isValid()) {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
            }
        }
    }
    //unsigned long duration = millis() - start;
    //Serial.printf("GPS update took %lu ms\n", duration);
}

void handleMenuNavigation(int buttonNumber) {
    if (cdo == 0 && buttonNumber == 3) { // Settings menu
        cdo = 99; // Special case for settings menu
        lastMenu[0] = "Callsign";
        lastMenu[1] = "Brightness";
        lastMenu[2] = "Bluetooth";
        lastMenu[3] = "Back";
        markDisplayForUpdate();
        return;
    }

    if (cdo == 99) { // In Settings menu
        handleSettingsNavigation(buttonNumber);
        return;
    }

    if (cdo == 98) { // Callsign editing
        handleCallsignEditing(buttonNumber);
        return;
    }
    switch (cdo) {
        case 0: {
            cdo = 1;
            menu = 0;
            lastMenu[0] = contacts[menu];
            lastMenu[1] = contacts[menu + 1];
            lastMenu[2] = contacts[menu + 2];
            lastMenu[3] = "Next...";
            markDisplayForUpdate();
            break;
        }
        case 1: {
            if (buttonNumber == 4) { // Next button
                menu = (menu + 3) % 9; // Cycle through contacts (wraps at 9 entries)
                lastMenu[0] = contacts[menu];
                lastMenu[1] = contacts[(menu + 1) % 9];
                lastMenu[2] = contacts[(menu + 2) % 9];
                lastMenu[3] = "Next...";
            } else {
                contact = contacts[menu + buttonNumber - 1];
                cdo++;
                menu = 0;
                lastMenu[0] = distances[menu];
                lastMenu[1] = distances[menu + 1];
                lastMenu[2] = distances[menu + 2];
                lastMenu[3] = "Next...";
            }
            markDisplayForUpdate();
            break;
        }
        case 2: {
            if (buttonNumber == 4) { // Next button
                menu = (menu + 3) % 9; // Cycle through distances (wraps at 9 entries)
                lastMenu[0] = distances[menu];
                lastMenu[1] = distances[(menu + 1) % 9];
                lastMenu[2] = distances[(menu + 2) % 9];
                lastMenu[3] = "Next...";
            } else {
                distance = distances[menu + buttonNumber - 1];
                cdo++;
                menu = 0;
                lastMenu[0] = orders[menu];
                lastMenu[1] = orders[menu + 1];
                lastMenu[2] = orders[menu + 2];
                lastMenu[3] = "Next...";
            }
            markDisplayForUpdate();
            break;
        }
        case 3: {
            if (buttonNumber == 4) { // Next button
                menu = (menu + 3) % 9; // Cycle through orders (wraps at 9 entries)
                lastMenu[0] = orders[menu];
                lastMenu[1] = orders[(menu + 1) % 9];
                lastMenu[2] = orders[(menu + 2) % 9];
                lastMenu[3] = "Next...";
            } else {
                order = orders[menu + buttonNumber - 1];
                newMsg = true;
                cdo = 0;
                menu = 0;
                lastMenu[0] = defaultMenu[0];
                lastMenu[1] = defaultMenu[1];
                lastMenu[2] = defaultMenu[2];
                lastMenu[3] = defaultMenu[3];
            }
            markDisplayForUpdate();
            break;
        }
    }
}

void checkButtons() {
    ezButton* buttons[] = {&button1, &button2, &button3, &button4};
    for (int i = 0; i < 4; i++) {
        buttons[i]->loop();
        if (buttons[i]->isPressed()) {
            //Serial.printf("Button %d pressed", i + 1);
            handleMenuNavigation(i + 1);
            markDisplayForUpdate();
        }
    }
}

void receivedCallback(uint32_t from, String &msg) {
    Serial.println(msg.c_str());
    incomingMsg = msg.c_str();
    lastMsgTime = millis();
    markDisplayForUpdate();
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup...");
    Serial.println("Initializing Serial2...");
    //Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    Serial.println("Setting debounce time...");
    button1.setDebounceTime(DEBOUNCE_TIME);
    button2.setDebounceTime(DEBOUNCE_TIME);
    button3.setDebounceTime(DEBOUNCE_TIME);
    button4.setDebounceTime(DEBOUNCE_TIME);

    Serial.println("Initializing display...");
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }

    Serial.println("Configuring display...");
    display.setTextWrap(true);
    display.dim(true);
    updateDisplay(defaultMenu[0], defaultMenu[1], defaultMenu[2], defaultMenu[3]);

    Serial.println("Initializing mesh...");
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
    mesh.setDebugMsgTypes(ERROR | CONNECTION);  

    Serial.println("Initializing Wire...");
    Wire.begin();
    Serial.println("Scanning I2C bus...");
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("I2C device found at address 0x");
            Serial.println(address, HEX);
        }
    }
    Serial.println("Scan complete");

    Serial.println("Configuring Compass...");
    if(!mag.begin())
    {
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }

    Serial.println("Adding tasks...");
    userScheduler.addTask(taskSendMessage);
    taskSendMessage.enable();

   // userScheduler.addTask(taskUpdateHeading);
    //taskUpdateHeading.enable();

    Serial.println("Setup completed!");
}

void loop() {
    //Serial.println("Checking buttons...");
    checkButtons();
    //Serial.println("Updating display if needed...");
    updateDisplayIfNeeded();
    //Serial.println("Updating mesh...");
    mesh.update();
    //Serial.println("Executing user scheduler...");
    userScheduler.execute();
    //Serial.println("Updating heading...");
    //updateHeading();
    //Serial.println("Updating GPS...");
    updateGPS();
    //Serial.println("Loop completed!");
}
