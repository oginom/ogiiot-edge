// reference
//MQTT with ESP32
//https://qiita.com/hilucky/items/0e394760a1445593cea5
//https://github.com/knolleary/pubsubclient
//STL for AVR
//https://qiita.com/kota65535/items/9349750c9b3a12910347
//http://andybrown.me.uk/wk/2011/01/15/the-standard-template-library-stl-for-avr-with-c-streams/

#include <WiFi.h>
#include <PubSubClient.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <vector>
using namespace std;

//// Edge ////

class Edge {
public:
    virtual void exec() {};
    virtual bool msg(char* s) {return false;};
    virtual bool rcv(char* cmd) {return false;};
};

class EchoSensor : public Edge {
public:
    int echoPin;
    int trigPin;
    EchoSensor(int _echoPin, int _trigPin) {
        echoPin = _echoPin;
        trigPin = _trigPin;
    }
    long dist;
    virtual void exec() {
        static int i;
        static long dur, distA;
        distA = 0;
        for (i=0 ; i < 10; i++) {
            digitalWrite(trigPin, LOW); // 初期化
            delayMicroseconds(2);
            digitalWrite(trigPin, HIGH); // 10mSのトリガーをモジュールに出す
            delayMicroseconds(10);
            digitalWrite(trigPin, LOW);
            dur = pulseIn(echoPin, HIGH);// echoピンから帰ってくる数値ｍSを読む
            distA  = distA + dur * 0.034/2;// 距離
        }
        dist = distA/10 ;   // 10回の平均値をアウトプット
    }
    virtual bool msg(char* s) {
        sprintf(s, "%d", dist);
        return true;
    }
};

class LEDArrayActuator : public Edge {
public:
    int rgbPin;
    Adafruit_NeoPixel strip;
    uint32_t col;
    int len;
    int len_next;
    bool isChanged;
    LEDArrayActuator(int _rgbPin) {
        rgbPin = _rgbPin;
        strip = Adafruit_NeoPixel(30, rgbPin, NEO_GRB + NEO_KHZ800);
        col = strip.Color(0,0,0);
        len = 30;
        isChanged = false;

        strip.begin();
        strip.show();
        delay(100);
        col = strip.Color(17,17,17);
        len_next = 10;
        updateLED();
    }
    virtual void exec() {
        if (isChanged) {
            updateLED();
            isChanged = false;
        }
    }
    virtual bool msg(char* s) {
        sprintf(s, "%d,%d,%d,%d", len,
            col >> 16 & 0xFF, col >> 8 & 0xFF, col & 0xFF);
        return true;
    }
    virtual bool rcv(char* cmd) {
        int x0, x1, x2;
        if (sscanf(cmd, "%d,%d,%d", &x0, &x1, &x2) == 3) {
            col = strip.Color(x0, x1, x2);
            isChanged = true;
        } else if (sscanf(cmd,"L,%d", &x0) == 1) {
            if (x0 < 0 || 30 <= x0) return false;
            len_next = x0;
            isChanged = true;
        } else {
            return false;
        }
        return true;
    }
    bool updateLED() {
        uint32_t black = strip.Color(0, 0, 0);
        if (len_next >= len) {
            //strip.updateLength(LEDlen);
        }
        for (int i = 0; i < len_next; ++i) {
            strip.setPixelColor(i, col);
        }
        for (int i = len_next; i < len; ++i) {
            strip.setPixelColor(i, black);
        }
        strip.show();
        strip.show();
        //strip.updateLength(LEDlen);
        len = len_next;
        return true;
    }
};

// WiFi
const char ssid[] = "WIFI_SSID";
const char passwd[] = "WIFI_PASSWORD";

// Pub/Sub
const char mqttHost[] = "192.168.XXX.XXX"; // MQTT host
const int mqttPort = 1883; // MQTT port
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//char cid[13] = "000000000000";
char pub_topic[] = "ogiiot/data/000000000000";
char sub_topic_all[] = "ogiiot/ctrl/all";
char sub_topic[] = "ogiiot/ctrl/000000000000";
char message[64];


vector<Edge*> edges = {};

void callback(char* topic, byte* payload, unsigned int length) {
    char cmd[length+1] = "";
    for (int i=0; i<length; i++) { cmd[i] = (char)payload[i]; }
    cmd[length] = 0;
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.println(cmd);

    if (strcmp(topic, sub_topic_all) == 0) {
      mqttClient.publish(pub_topic, "hello");
    } else if (strcmp(topic, sub_topic) == 0) {
      edges[0]->rcv(cmd);
    }
}

void setup() {

    vector<int> OUT_PINS = {13, 25};
    vector<int> IN_PINS = {26};
    for (int i = 0; i < OUT_PINS.size(); ++i) pinMode(OUT_PINS[i], OUTPUT);
    for (int i = 0; i < IN_PINS.size(); ++i) pinMode(IN_PINS[i], INPUT);

    //Edge *ec = new EchoSensor(26, 25);
    //edges.push_back(ec);
    Edge *la = new LEDArrayActuator(13);
    edges.push_back(la);

    uint64_t chipid=ESP.getEfuseMac();
    char cid[13];
    sprintf(cid, "%04X%08X",(uint16_t)(chipid>>32),(uint32_t)chipid);
    sprintf(pub_topic, "ogiiot/data/%s", cid);
    sprintf(sub_topic, "ogiiot/ctrl/%s", cid);

    Serial.begin(9600);

    // Connect WiFi
    connectWiFi();

    // Connect MQTT
    connectMqtt();
    
    Serial.println("setup complete");
}

void loop() {
    static bool online = true;
    for (vector<Edge*>::iterator it = edges.begin(); it != edges.end(); ++it) {
        (*it)->exec();
    }

    //publish every 1s
    static unsigned long t = 0;
    if (millis() > t + 1000) {
        t = millis();

        for (vector<Edge*>::iterator it = edges.begin(); it != edges.end(); ++it) {
            char msg[20];
            if ((*it)->msg(msg)) {
                mqttClient.publish(pub_topic, msg);
            }
        }
    }

    // WiFi
    if ( WiFi.status() == WL_DISCONNECTED ) { connectWiFi(); }
    // MQTT
    if ( ! mqttClient.connected() ) { connectMqtt(); }
    mqttClient.loop();
}

/**
 * Connect WiFi
 */
void connectWiFi()
{
    WiFi.begin(ssid, passwd);
    Serial.print("WiFi connecting...");
    while(WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.print(" connected. ");
    Serial.println(WiFi.localIP());
}
/**
 * Connect MQTT
 */
void connectMqtt()
{
    mqttClient.setServer(mqttHost, mqttPort);
    mqttClient.setCallback(callback);
    while( ! mqttClient.connected() ) {
        Serial.println("Connecting to MQTT...");
        String clientId = "ESP32-" + String(random(0xffff), HEX);
        if ( mqttClient.connect(clientId.c_str()) ) {
            Serial.println("connected");
            mqttClient.subscribe(sub_topic_all);
            mqttClient.subscribe(sub_topic);
        }
        delay(1000);
        randomSeed(micros());
    }
}
