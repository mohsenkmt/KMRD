//KMRD PROJECT FINAL

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Wire.h>



const char *ssid = "KMRD Project";
const char *password = "KMRDmoreKD";
const char *controllerHTML = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>Quadcopter Controller</title>
    <style>
      body {
        font-family: Arial, sans-serif;
        text-align: center;
        margin: 0;
        padding: 0;
        background: #f2f2f2;
      }
      .joystick { 
        width: 300px; 
        height: 300px; 
        border-radius: 50%; 
        background: #ddd; 
        position: relative; 
        margin: 65px; 
        display: inline-block; 
    }
      .knob { 
        width: 85px; 
        height: 85px;
        border-radius: 50%; 
        background: #666; 
        position: absolute; 
        top: 50%; 
        left: 50%; 
        transform: translate(-50%, -50%); 
        transition: 0.05s; 
    }
    .button { 
        padding: 35px 35px; 
        margin-top: 200px;
        margin-left: 5px;
        margin-right: 5px; 
        background: #65a7ed; 
        color: white; 
        border: none; 
        border-radius: 5px; 
        cursor: pointer; 
    }
      .status {
        font-size: 16px;
        margin-top: 20px;
      }
    table,td,th{
      border: 1px solid;
      border-collapse: collapse;
      
    }

    #PID{
      margin-left: 65px;
    }
    #info{
      margin-left: 65px;
    }
    #quadcopterInfo{
      display: flex;
    }
    </style>
  </head>
  <body>
    <div class="status">
      Battery: <span id="battery">100%</span> | Signal: <span id="signal">Strong</span>
    </div>
    <div style="display: flex; justify-content: center;">
      <div class="joystick" id="joystickL"><div class="knob" id="knobL"></div></div>
      <div>
        <button class="button" id="home">LED</button>
        <button class="button" id="camera">Camera</button>
        <button class="button" id="force_stop">Stop</button>
      </div>
      <div class="joystick" id="joystickR"><div class="knob" id="knobR"></div></div>
    </div>

    <div id="quadcopterInfo" style="margin-top: 5px; font-size: 18px; justify-items: center;">
  
      <table id="info" >
        <thead>
          <th>Motor Speeds</th>
          <th>Angles</th>
          <th>Accelerations</th>
          <th>Temperature</th>
        </thead>
        <tbody>
          <td align="center">
            <table border="1" style="text-align: center;">
              <thead>
                <th>Num</th>
                <th>Speed</th>
              </thead>
              <tbody>
                <tr>
                  <td>1</td>
                  <td><span id="motor1Speed"> 0 </span></td>
                </tr>
                <tr>
                  <td>2</td>
                  <td><span id="motor2Speed"> 0 </span></td>
                </tr>
                <tr>
                  <td>3</td>
                  <td><span id="motor3Speed"> 0 </span></td>
                </tr>
                <tr>
                  <td>4</td>
                  <td><span id="motor4Speed"> 0 </span></td>
                </tr>

              </tbody>
            </table>
          </td>

          <td align="center">
              <table border="1" style="height: 100px; width: 50px; text-align: center;">
                <thead>
                  <th>Var</th>
                  <th>Value</th>
                </thead>
                <tbody>
                  <tr>
                    <td>X</td> 
                    <td><span id="angleX">0</span></td> 
                      
                  </tr>
                  <tr>
                    <td>Y</td> 
                    <td><span id="angleY">0</span></td>
                  </tr>
                </tbody>
                
              </table>
          </td>

          <td align="center">
            <table border="1" style="text-align: center;">
              <thead>
                <th>Var</th>
                <th>Value</th>
              </thead>
              <tbody>
                <tr>
                  <td>X</td>
                  <td><span id="accX">0</span> </td>
                </tr>
                <tr>
                  <td>Y</td>
                  <td><span id="accY">0</span> </td>
                </tr>
                <tr>
                  <td>Z</td>
                  <td><span id="accZ">0</span></td>
                </tr>
              </tbody>
            </table>
          </td>
          <td>
            <span id="temp">0</span><sup>°</sup>c
          </td>
        </tbody>
    </table>
  
    <table id="PID">
      <thead>
        <th colspan="3">PID Values</th>
        <th colspan="3">Motors Values</th>
      </thead>
      <tbody>
        <tr>
          <th style="width: 65px;">P (k<sub>P</sub>)</th>
          <td style="width: 265px;"><input type="text" id="P"> <input type="submit" onclick="save_PID()"></td>
          <td style="width: 65px;"><span id="P_value">0</span></td>

          <th style="width: 65px;">M<sub>1</sub></th>
          <td style="width: 265px;"><input type="text" id="M1"> <input type="submit" onclick="save_PID()"></td>
          <td style="width: 65px;"><span id="M1_value">0</span></td>
        </tr>
        <tr>
          <th style="width: 65px;">I (k<sub>I</sub>)</th>
          <td style="width: 265px;"><input type="text" id="I" > <input type="submit" onclick="save_PID()"></td>
          <td style="width: 65px;"><span id="I_value" >0</span></td>

          <th style="width: 65px;">M<sub>2</sub></th>
          <td style="width: 265px;"><input type="text" id="M2"> <input type="submit" onclick="save_PID()"></td>
          <td style="width: 65px;"><span id="M2_value">0</span></td>
        </tr>
        <tr>
          <th style="width: 165px;">D (k<sub>D</sub>)</th>
          <td style="width: 265px;"><input type="text" id="D"> <input type="submit" onclick="save_PID()"></td>
          <td style="width: 65px;"><span id="D_value">0</span></td>

          <th style="width: 65px;">M<sub>3</sub></th>
          <td style="width: 265px;"><input type="text" id="M3"> <input type="submit" onclick="save_PID()"></td>
          <td style="width: 65px;"><span id="M3_value">0</span></td>
        </tr>
        <tr>
          <td colspan="3"></td>
          
          <th>M<sub>4</sub></th>
          <td style="width: 265px;"><input type="text" id="M4"> <input type="submit" onclick="save_PID()"></td>
          <td style="width: 65px;"><span id="M4_value">0</span></td>

        </tr>
      </tbody>
    </table>
  
    </div>
    <script>
      var cam_state=false;
      var ret_home_state=false;
      var fstop=false;
      var currentP=0;
      var currentI=0;
      var currentD=0;

      var currentm1=0;
      var currentm2=0;
      var currentm3=0;
      var currentm4=0;

      const knobL = document.getElementById('knobL');
      const knobR = document.getElementById('knobR');
      const homeBtn = document.getElementById('home');
      const cameraBtn = document.getElementById('camera');
      const stopBtn = document.getElementById('force_stop');

      let joystickLData = { x: 512, y: 512 };
      let joystickRData = { x: 512, y: 512 };

      // Send data to ESP32
      function sendData() {
        fetch(`/update?lx=${joystickLData.x}&ly=${joystickLData.y}&rx=${joystickRData.x}&ry=${joystickRData.y}`);
      }

      function fetchQuadcopterInfo() {
        fetch('/info') // فرض بر این است که ESP32 اطلاعات را در مسیر `/info` ارسال می‌کند
          .then(response => response.json())
          .then(data => {
            // به‌روزرسانی اطلاعات موتور
            document.getElementById('motor1Speed').innerText = data.motor1;
            document.getElementById('motor2Speed').innerText = data.motor2;
            document.getElementById('motor3Speed').innerText = data.motor3;
            document.getElementById('motor4Speed').innerText = data.motor4;
      
            // به‌روزرسانی زاویه‌ها
            document.getElementById('angleX').innerText = data.angleX;
            document.getElementById('angleY').innerText = data.angleY;
      
            // به‌روزرسانی شتاب‌ها
            document.getElementById('accX').innerText = data.accX;
            document.getElementById('accY').innerText = data.accY;
            document.getElementById('accZ').innerText = data.accZ;
            
            document.getElementById('temp').innerText = data.temp;


            document.getElementById('P_value').innerText = data.Pv;
            currentP=data.Pv;
            document.getElementById('I_value').innerText = data.Iv;
            currentI=data.Iv;
            document.getElementById('D_value').innerText = data.Dv;
            currentD=data.Dv;

            document.getElementById('M1_value').innerText = data.M1v;
            currentm1=data.M1v;
            document.getElementById('M2_value').innerText = data.M2v;
            currentm2=data.M2v;
            document.getElementById('M3_value').innerText = data.M3v;
            currentm3=data.M3v;
            document.getElementById('M4_value').innerText = data.M4v;
            currentm4=data.M4v;


            document.getElementById('battery').innerText = data.battery;
            document.getElementById('signal').innerText = data.signal;
          })
          .catch(err => console.error('Failed to fetch quadcopter info:', err));
      }
      
      // دریافت اطلاعات هر 500 میلی‌ثانیه
      setInterval(fetchQuadcopterInfo, 500);
      
      //change button value
      function changeState(val){
          if(val==="camera"){ 
            cam_state= !cam_state;
            fetch(`/camera?cam=${cam_state}`);
            }
          if(val==="home"){
            ret_home_state= !ret_home_state;
            fetch(`/home?rethome=${ret_home_state}`);
          } 
          if(val==="force_stop"){
            fstop= !fstop;
            fetch(`/fstop?forcestop=${fstop}`);
            if(fstop){
              document.getElementById("force_stop").style="background: #c62540;"
            }else{
              document.getElementById("force_stop").style="background: #65a7ed"
            }
          }
          
      }


      // Move joystick knob
      function moveKnob(knob, data, joystick) {
        const rect = joystick.getBoundingClientRect();
        const x = ((data.x - 512) / 1023) * rect.width / 2;
        const y = ((data.y - 512) / 1023) * rect.height / 2;
        knob.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
      }

      // Keyboard control
      const keyState = {}; // To track the state of pressed keys

    document.addEventListener('keydown', (e) => {
      keyState[e.key] = true; // Mark the key as pressed
      updateJoystickPosition(); // Update joystick positions based on key states
    });

    document.addEventListener('keyup', (e) => {
      keyState[e.key] = false; // Mark the key as released
      updateJoystickPosition(); // Update joystick positions based on key states
    });

    // Function to update joystick positions
    function updateJoystickPosition() {
      // Left joystick movement (joystickL)
      joystickLData.y = keyState['w'] ? Math.max(joystickLData.y - 25, 0) :
                        keyState['s'] ? Math.min(joystickLData.y + 25, 1023) : 512;

      // Right joystick movement (joystickR)
      joystickRData.x = keyState['ArrowLeft'] ? Math.max(joystickRData.x - 25, 0) :
                          keyState['ArrowRight'] ? Math.min(joystickRData.x + 25, 1023) : 512;

      joystickRData.y = keyState['ArrowUp'] ? Math.max(joystickRData.y - 25, 0) :
                          keyState['ArrowDown'] ? Math.min(joystickRData.y + 25, 1023) : 512;

      // Visualize the changes on the joystick knobs
      moveKnob(knobL, joystickLData, document.getElementById('joystickL'));
      moveKnob(knobR, joystickRData, document.getElementById('joystickR'));

      // Send updated data
      sendData();
    }

      
      function save_PID()
      {
        
        var P = document.getElementById('P').value || currentP;
        var I = document.getElementById('I').value || currentI;
        var D = document.getElementById('D').value || currentD;


        var M1 = document.getElementById('M1').value || currentm1;
        var M2 = document.getElementById('M2').value || currentm2;
        var M3 = document.getElementById('M3').value || currentm3;
        var M4 = document.getElementById('M4').value || currentm4;
        
        fetch(`/PIDValue?Pval=${P}&Ival=${I}&Dval=${D}&m1val=${M1}&m2val=${M2}&m3val=${M3}&m4val=${M4}`);

        document.getElementById('P').value="";
        document.getElementById('I').value="";
        document.getElementById('D').value="";
        document.getElementById('M1').value="";
        document.getElementById('M2').value="";
        document.getElementById('M3').value=""; 
        document.getElementById('M4').value="";

        alert("PID & Motors value Succesfuly Updated .");
      }

      // Touch control for mobile
      function handleTouch(joystick, knob, data) {
        joystick.addEventListener('touchmove', (e) => {
          const rect = joystick.getBoundingClientRect();
          const touch = e.touches[0];
          const x = Math.min(Math.max((touch.clientX - rect.left) / rect.width, 0), 1);
          const y = Math.min(Math.max((touch.clientY - rect.top) / rect.height, 0), 1);

          data.x = Math.floor(x * 1023);
          data.y = Math.floor(y * 1023);

          moveKnob(knob, data, joystick);
          sendData();
        });

        joystick.addEventListener('touchend', () => {
          data.x = 512;
          data.y = 512;
          moveKnob(knob, data, joystick);
          sendData();
        });
      }

      // Attach touch handlers
      handleTouch(document.getElementById('joystickL'), knobL, joystickLData);
      handleTouch(document.getElementById('joystickR'), knobR, joystickRData);

      // Button clicks
      homeBtn.addEventListener('click', () => changeState("home"));
      cameraBtn.addEventListener('click', () => changeState("camera"));
      stopBtn.addEventListener('click', () => changeState("force_stop"));
    </script>
  </body>
</html>
)rawliteral";


//Motors
#define MOTOR1_PIN 14  //FL
#define MOTOR2_PIN 27  //FR
#define MOTOR3_PIN 13  //BL
#define MOTOR4_PIN 12  //BR
#define MIN_THROTTLE 30
#define MAX_THROTTLE 255


#define REF_VOLTAGE 8 




// Motors Speed
int motor1Speed = 0;
int motor2Speed = 0;
int motor3Speed = 0;
int motor4Speed = 0;
int currentThrottle1 = 0;
int currentThrottle2 = 0;
int currentThrottle3 = 0;
int currentThrottle4 = 0;
int joystickLx = 0, joystickLy = 0, joystickRx = 0, joystickRy = 0;
int LedPin = 2;
int CameraPin = 15;
int batteryPin = 34;


int signal_info=100;
int battery_info=50;


float Rate_x, Rate_y, Rate_z;
float AccX, AccY, AccZ;
float Angle_x, Angle_y;
float LoopTimer;

float kp = 1.0;  // ضریب تناسبی
float ki = 0.01;  // ضریب انتگرالی
float kd = 0.1;  // ضریب مشتقی


float mt1value=0;
float mt2value=0;
float mt3value=0;
float mt4value=0;



float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;
float prevRollError = 0, prevPitchError = 0, prevYawError = 0;

// Variables for gradual changes
float currentRollTarget = 0;  // Smooth roll target
float currentPitchTarget = 0; // Smooth pitch target
float currentYawTarget = 0;   // Smooth yaw target
float throttleSmoothing = 0.05; // Factor for throttle smoothing (0.01 - 1)
float angleSmoothing = 0.1;    // Factor for angle smoothing (0.01 - 1)



bool cameraStatus = false;
bool returnHome = false;
bool isStarted = false;
bool forceStop = false;

float temperature;



//Waeb Server
AsyncWebServer server(80);

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void init_all() {
  // Motors
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);

  //Led
  pinMode(LedPin,OUTPUT);

  //Camera
  pinMode(CameraPin,OUTPUT);


}


void toggles() {
  if (cameraStatus) digitalWrite(CameraPin, HIGH);
  else digitalWrite(CameraPin, LOW);

  if (returnHome) digitalWrite(LedPin,HIGH);
  else digitalWrite(LedPin, LOW);

  if (forceStop){
    isStarted=false;
  }
  delay(10);


}


void setMotorSpeeds(int m1, int m2, int m3, int m4) {
  motor1Speed = constrain(m1, 0, MAX_THROTTLE);
  motor2Speed = constrain(m2, 0, MAX_THROTTLE);
  motor3Speed = constrain(m3, 0, MAX_THROTTLE);
  motor4Speed = constrain(m4, 0, MAX_THROTTLE);

  analogWrite(MOTOR1_PIN, motor1Speed);
  analogWrite(MOTOR2_PIN, motor2Speed);
  analogWrite(MOTOR3_PIN, motor3Speed);
  analogWrite(MOTOR4_PIN, motor4Speed);
}


void TestMotors() {
  setMotorSpeeds(MAX_THROTTLE, 0, 0, 0);
  delay(10);
  setMotorSpeeds(0, MAX_THROTTLE, 0, 0);
  delay(10);
  setMotorSpeeds(0, 0, MAX_THROTTLE, 0);
  delay(10);
  setMotorSpeeds(0, 0, 0, MAX_THROTTLE);
  delay(10);
  setMotorSpeeds(0, 0, 0, 0);
}

void readTemperature() {
  Wire.beginTransmission(0x68);
  Wire.write(0x41); 
  Wire.endTransmission(false); 
  
  Wire.requestFrom(0x68, 2); 
  while (Wire.available() < 2); 

  int16_t temp = Wire.read() << 8 | Wire.read();   
  temperature=(temp / 340.0) + 36.53;
  delay(10);
}

void getAngle(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  Rate_x = (float)GyroX / 65.5;
  Rate_y = (float)GyroY / 65.5;
  Rate_z = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  AccZ = AccZ - 0.26;  // calibration offset
  Angle_x = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  Angle_y = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
  delay(10);
}

void MPU6050_Setup() {
  Wire.setClock(400000);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

float readBatteryPercentage() {
    
    // خواندن مقدار ADC
    int adcValue = analogRead(batteryPin);

    // تبدیل ADC به ولتاژ
    float batteryVoltage = adcValue * (8.4 / 4095.0); // مقیاس‌گذاری 12 بیتی

    // تبدیل ولتاژ به درصد
    float batteryPercentage = (batteryVoltage - 7.0) / (8.4 - 7.0) * 100;

    // محدود کردن درصد بین 0 تا 100
    batteryPercentage = constrain(batteryPercentage, 0, 100);

    return batteryPercentage;
}


void WifiSetup() {
  WiFi.softAP(ssid, password);
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP().toString());

  delay(1000);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", controllerHTML);
  });


  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("lx")) joystickLx = map(request->getParam("lx")->value().toInt(), 0, 1023, -100, 100);
    if (request->hasParam("ly")) joystickLy = map(request->getParam("ly")->value().toInt(), 0, 1023, 100, -100);
    if (request->hasParam("rx")) joystickRx = map(request->getParam("rx")->value().toInt(), 0, 1023, -100, 100);
    if (request->hasParam("ry")) joystickRy = map(request->getParam("ry")->value().toInt(), 0, 1023, 100, -100);
    request->send(200, "text/plain", "OK");});

  server.on("/camera", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("cam")) {
      String camValue = request->getParam("cam")->value();
      cameraStatus = camValue.equalsIgnoreCase("true") || camValue.equals("1");
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/fstop", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("forcestop")) {
      String fStop = request->getParam("forcestop")->value();
      forceStop = fStop.equalsIgnoreCase("true") || fStop.equals("1");
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/home", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("rethome")) {
      String homeValue = request->getParam("rethome")->value();
      returnHome = homeValue.equalsIgnoreCase("true") || homeValue.equals("1");
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/PIDValue", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("Pval")) kp=request->getParam("Pval")->value().toFloat();
    if (request->hasParam("Ival")) ki=request->getParam("Ival")->value().toFloat();
    if (request->hasParam("Dval")) kd=request->getParam("Dval")->value().toFloat();

    if (request->hasParam("m1val")) mt1value=request->getParam("m1val")->value().toFloat();
    if (request->hasParam("m2val")) mt2value=request->getParam("m2val")->value().toFloat();
    if (request->hasParam("m3val")) mt3value=request->getParam("m3val")->value().toFloat();
    if (request->hasParam("m4val")) mt4value=request->getParam("m4val")->value().toFloat();

    request->send(200, "text/plain", "OK");
  });
  

  server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"motor1\":" + String(motor1Speed) + ",";
    json += "\"motor2\":" + String(motor2Speed) + ",";
    json += "\"motor3\":" + String(motor3Speed) + ",";
    json += "\"motor4\":" + String(motor4Speed) + ",";
    json += "\"angleX\":" + String(Angle_x) + ",";
    json += "\"angleY\":" + String(Angle_y) + ",";
    json += "\"accX\":" + String(AccX) + ",";
    json += "\"accY\":" + String(AccY) + ",";
    json += "\"accZ\":" + String(AccZ) + ",";

    json += "\"temp\":" + String(temperature) + ",";

    json += "\"Pv\":" + String(kp,10) + ",";
    json += "\"Iv\":" + String(ki,10) + ",";
    json += "\"Dv\":" + String(kd,10) + ",";

    json += "\"M1v\":" + String(mt1value) + ",";
    json += "\"M2v\":" + String(mt2value) + ",";
    json += "\"M3v\":" + String(mt3value) + ",";
    json += "\"M4v\":" + String(mt4value) + ",";

    json += "\"battery\":" + String(readBatteryPercentage()) + ",";
    json += "\"signal\":" + String(getWiFiSignalStrength());
    json += "}";

    request->send(200, "application/json", json);
  });

  server.onNotFound(notFound);
  server.begin();
}


int getWiFiSignalStrength() {
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.RSSI(); // دریافت قدرت سیگنال
  } else {
    return -100; // مقدار پیش‌فرض در صورت عدم اتصال
  }
}

// Ensure motors stay off until user starts
float hoverThrottle = 45; // Base throttle for maintaining height (adjust as needed)
float throttleIncrement = 5; // Step size for throttle increase/decrease

void updatePID() {
  //readTemperature();
  getAngle();
  // Check if motors should start
  if (!isStarted) {
    if (joystickLy > 0) { // Start when left joystick is moved
      isStarted = true;
    } else {
      setMotorSpeeds(0, 0, 0, 0); // Motors off
      rollIntegral = pitchIntegral = yawIntegral = 0; // Reset integrals
      prevRollError = prevPitchError = prevYawError = 0; // Reset errors
      return;
    }
  }

  // Adjust base throttle based on joystickLy

  if (joystickLy > 0) { // Increase throttle
    hoverThrottle += throttleIncrement * (joystickLy / 100.0); // Scale by joystick input
  } else if (joystickLy < 0) { // Decrease throttle
    hoverThrottle += throttleIncrement * (joystickLy / 100.0); // Scale by joystick input
  }
  hoverThrottle = constrain(hoverThrottle, MIN_THROTTLE + 3, MAX_THROTTLE - 3); // Prevent motor cutoff

  // Desired angles and yaw rate from joystick inputs
  float targetRoll = map(joystickRy, -100, 100, -15, 15);   // Desired roll angle
  float targetPitch = map(joystickRx, -100, 100, -15, 15);  // Desired pitch angle
  float targetYawRate = map(joystickLx, -100, 100, -100, 100); // Desired yaw rate

  // Gradually update targets for smoother transitions
  currentRollTarget += angleSmoothing * (targetRoll - currentRollTarget);
  currentPitchTarget += angleSmoothing * (targetPitch - currentPitchTarget);
  currentYawTarget += angleSmoothing * (targetYawRate - currentYawTarget);

  // Errors for PID
  float rollError = currentRollTarget - Angle_x;  // Roll error
  float pitchError = currentPitchTarget - Angle_y; // Pitch error
  float yawError = currentYawTarget - Rate_z; // Yaw error

  float deadZone = 2.0;
  if (abs(rollError) < deadZone) rollError = 0;
  if (abs(pitchError) < deadZone) pitchError = 0;
  if (abs(yawError) < deadZone) yawError = 0;


  // Roll PID calculation
  rollIntegral += rollError * LoopTimer;
  float rollDerivative = (rollError - prevRollError) / LoopTimer;
  float rollOutput = kp * rollError + ki * rollIntegral + kd * rollDerivative;

  // Pitch PID calculation
  pitchIntegral += pitchError * LoopTimer;
  float pitchDerivative = (pitchError - prevPitchError) / LoopTimer;
  float pitchOutput = kp * pitchError + ki * pitchIntegral + kd * pitchDerivative;

  // Yaw PID calculation
  yawIntegral += yawError * LoopTimer;
  float yawDerivative = (yawError - prevYawError) / LoopTimer;
  float yawOutput = kp * yawError + ki * yawIntegral + kd * yawDerivative;

  // Save previous errors for next calculation
  prevRollError = rollError;
  prevPitchError = pitchError;
  prevYawError = yawError;

  int m1Adjust=0;
  int m2Adjust=0;
  int m3Adjust=0;
  int m4Adjust=0;
  if(joystickRx!=0||joystickRy!=0){

    // Adjust motor speeds based on PID outputs
    m1Adjust = -rollOutput - pitchOutput + yawOutput; // Front Left
    m2Adjust = rollOutput - pitchOutput - yawOutput;  // Front Right
    m3Adjust = -rollOutput + pitchOutput - yawOutput; // Back Left
    m4Adjust = rollOutput + pitchOutput + yawOutput;  // Back Right

  }
  else{

    // Adjust motor speeds based on PID outputs
    m1Adjust = rollOutput + pitchOutput - yawOutput; // Front Left
    m2Adjust = -rollOutput + pitchOutput + yawOutput; // Front Right
    m3Adjust = rollOutput - pitchOutput + yawOutput; // Back Left
    m4Adjust = -rollOutput - pitchOutput - yawOutput; // Back Right
  }
  

  // Calculate final motor speeds
  int m1 = hoverThrottle + m1Adjust;
  int m2 = hoverThrottle + m2Adjust;
  int m3 = hoverThrottle + m3Adjust;
  int m4 = hoverThrottle + m4Adjust;

  if((m1<10 && m2<10 && m3<10 && m4<10) && joystickLy<0){
    isStarted=false;
  }

  // Constrain motor speeds to valid range
  m1 = constrain((m1+mt1value), MIN_THROTTLE, MAX_THROTTLE);
  m2 = constrain((m2+mt2value), MIN_THROTTLE, MAX_THROTTLE);
  m3 = constrain((m3+mt3value), MIN_THROTTLE, MAX_THROTTLE);
  m4 = constrain((m4+mt4value), MIN_THROTTLE, MAX_THROTTLE);

  // Apply motor speeds
  setMotorSpeeds(m1, m2, m3, m4);
  delay(10);
}

void setup() {
  // KMRD PROJECT SETUP

  init_all();
  WifiSetup();
  Wire.begin();
  MPU6050_Setup();
  Serial.begin(115200);
  TestMotors();
  delay(10);

}

void loop() {

  getAngle();
  readTemperature();
  
  static unsigned long lastTime = 0;
  unsigned long currentTime = micros();
  LoopTimer = (currentTime - lastTime) / 1000000.0; // Time in seconds
  lastTime = currentTime;

  // Update PID logic
  updatePID();
  // move_haandler();
  toggles();
  

  delay(10);

}