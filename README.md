# 프로젝트 명
웹캠을 이용해 객체를 피해가는 라인트레이서

# 개발기간
2023.04.24 ~ 2023.04.28

# 참여인원
3 명

# 개발환경
- Windows 10
- C / C++
- Arduino IDE 2.1 라이브러리 버전
- Python 3.11
- Visual Studio Code
- PyCharm
- openCV 라이브러리
- YOLOv5 객체 탐지 모델
- DRGO 웹캠
- 라인 트레이서(Arduino Nano, L9110S 모터 드라이버, IR 센서, 초음파 센서)

# 주요기능
## 1. 웹캠을 이용한 객체 탐지
- YOLOv5 모델
![a0](/img/a0.png)
- YOLOv5 모델을 이용해서 웹캠 영상 내의 객체들을 탐지 후 'person' 객체만 인식해서 화면 출력 및 라인트레이서에 객체 위치 정보 전송.
![a1](/img/a1.png)
![a2](/img/a2.png)
- 이미지 변환

![b1](/img/b1.png)
![b2](/img/b2.png)
![b3](/img/b3.png)
- 영상을 그레이 이미지로 변환 후, Threshold 기능을 통해 흑백 이미지로 변환 후 값을 반전시킨 후 가장 큰 영역을 라인으로 인식.

## 2. 웹캠과 아두이노 간의 시리얼 통신
- 웹캠은 라인 트레이서와 실시간으로 통신을 하며, 객체가 없을 시 D, 객체가 있을 시 T 신호를 전송
- 웹캠은 라인 트레이서와 실시간으로 통신을 하며, 전방에 있는 라인의 중심점 위치에 따라서 L(좌), R(우), F(전방) 신호 전송

## 3. 딜레이 기능을 통한 라인트레이서의 속도 조절
- analogWrite가 정상 동작하지 않는 관계로 delay함수를 통해 모터를 제어하여 원하는 동작을 수행.

## 4. 객체 탐지 시 저장된 동작 수행
- 라인 내의 객체를 탐지하고 라인에서 벗어난 뒤 일련의 사이클을 수행 한 후 다시 라인 방향으로 주행.
- 사이클 수행 후 라인으로 도착했을 경우, 라인 트레이싱 수행.
- 사이클 수행 후 라인으로 도착하지 못했을 경우, 웹캠으로부터 전송받은 라인 중심점 정보를 따라 자동 주행.
- 자동 주행 후 라인으로 도착했을 경우, 라인 트레이싱 수행 이하 반복.

# 작업시간
33시간

# 주요 코드

## 1. Arduino
```
void loop() {
  //IR 센서 값을 읽어 출력해주는 코드
  // IR_L_data = analogRead(IR_L);
  // IR_M_data = analogRead(IR_M);
  // IR_R_data = analogRead(IR_R);
  IR_L_data = digitalRead(IR_L);
  IR_M_data = digitalRead(IR_M);
  IR_R_data = digitalRead(IR_R);

  // Serial.println((String)IR_L_data +"-"+IR_M_data+"-"+IR_R_data);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;

  if (distance < 15) {
    turn();
  } else {
    if (IR_L_data == 0 and IR_M_data == 1 and IR_R_data == 0) {
      forward();
    } else if (IR_L_data == 1 and IR_M_data == 0 and IR_R_data == 0) {
      left();
    } else if (IR_L_data == 0 and IR_M_data == 0 and IR_R_data == 1) {
      right();
    } else if (IR_L_data == 1 and IR_M_data == 1 and IR_R_data == 0) {
      smooth_left();
    } else if (IR_L_data == 0 and IR_M_data == 1 and IR_R_data == 1) {
      smooth_right();
    } else if (IR_L_data == 1 and IR_R_data == 1) {
      stop();
    }
  }
}

void turn() {
  millifirstleft(500);
  milliforward(1200);
  milliright(500);
  milliforward(1500);
  milliright(500);
  milliforward(700);
  millilastforward();
}

void milliright(unsigned long x) {
  unsigned long current_time = millis();
  unsigned long interval = x;
  while (millis() - current_time < interval) {
    hardright();
  }
}

void millilastforward() {
  while (IR_L_data == 0 and IR_L_data == 0 and IR_L_data == 0) {
    forward();
    if (IR_L_data == 1 or IR_M_data == 1 or IR_R_data == 1) {
      stop();
      break;
    }
  }
  millilastleft(200);
}

void millilastleft(unsigned long x) {
  unsigned long current_time = millis();
  unsigned long interval = x;
  while (millis() - current_time < interval) {
    hardleft();
  }
  left();
  if (IR_L_data == 1 or IR_M_data == 1 or IR_R_data == 1) {
    return;
  }
}
```

## 2. Python
### 1.
```
import cv2
import numpy as np
import torch
import serial

ser = serial.Serial('COM4', 9600)

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

cap = cv2.VideoCapture(0)  # 0번 카메라 연결

person_detected = False  # person 객체가 감지되었는지 여부를 저장하는 변수

while True:
    ret, frame = cap.read()  # 프레임 읽기
    if not ret:
        break
    results = model(frame)  # 객체 감지 수행

    for detection in results.xyxy[0]:
        # 객체의 위치와 클래스 정보를 가져옵니다.
        x1, y1, x2, y2, conf, cls = detection
        label = model.names[int(cls)]

        # person 객체일 때만 그리기
        if label == 'person':
            person_detected = True  # person 객체가 감지되었다고 표시
            # 객체의 위치에 사각형을 그려줍니다.
            cv2.rectangle(frame, (int(x1), int(y1)),
                          (int(x2), int(y2)), (0, 255, 0), 2)
            # 객체의 클래스 이름과 정확도를 화면에 출력합니다.
            cv2.putText(frame, f'{label} {conf:.2f}', (int(x1), int(y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 0), 2)
```
### 2.
```
    # 하단부 중앙 1/2 영역 추출하기
    crop_width = int(width / 4)
    crop_height = int(height / 2)
    start_x = int(width / 2) - crop_width
    start_y = int(height / 2)
    end_x = int(width / 2) + crop_width
    end_y = height
    cropped_frame = frame[start_y:end_y, start_x:end_x]

    # 그레이스케일 이미지로 변환하기
    gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)

    # 이미지 이진화하기
    _, thresh = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)

    # 컨투어 찾기
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area and area > min_contour_area:
            max_area = area
            max_contour = contour

    # 가장 큰 영역의 중심점 찾기
    if max_contour is not None:
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 중심점 그리기
            cv2.circle(thresh, (cx, cy), 5, (0, 0, 255), -1)

            # 중심점 정보 시리얼 통신으로 전송하기
            if cx < 210:
                ser.write(b'L')
            elif cy > 450:
                ser.write(b'R')
            else:
                ser.write(b'F')
```
## 3. Arduino 2
```
void loop() {
  while (Serial.available() > 0) {
    char input = Serial.read();

    if (input == 'F') {
      forward();
      delay(75);
      stop();
    } else if (input == 'B') {
      backward();
      delay(75);
      stop();
    } else if (input == 'L') {
      left();
      delay(75);
      stop();
    } else if (input == 'R') {
      right();
      delay(75);
      stop();
    } else{
      stop();
    }
  }
}
```
## 4. Arduino 3
```
BluetoothSerial SerialBT;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(115200);
  SerialBT.begin("ESP32-CAM");
  Serial.println("Bluetooth device is ready to pair");
}

void loop() {
  if (SerialBT.available()) {
    String command = SerialBT.readString();
    int num = 0;

    if (command.equals("F")) {
      num = 1;
    } else if (command.equals("L")) {
      num = 2;
    } else if (command.equals("R")) {
      num = 3;
    } else if (command.equals("B")) {
      num = 4;
    } else if (command.equals("S")) {
      num = 5;
    }

    switch (num) {
      case 1:
        // 전진 명령 처리 코드
        forward();
        break;
      case 2:
        // 좌회전 명령 처리 코드
        left();
        break;
      case 3:
        // 우회전 명령 처리 코드
        right();
        break;
      case 4:
        // 후진 명령 처리 코드
        backward();
        break;
      case 5:
        // 정지 명령 처리 코드
        stop();
        break;
    }
  }
}

```
## 5. Android
```

void bluetoothOn() {
        if(mBluetoothAdapter == null) {
            Toast.makeText(getApplicationContext(), "블루투스를 지원하지 않는 기기입니다.", Toast.LENGTH_LONG).show();
        }
        else {
            if (mBluetoothAdapter.isEnabled()) {
                Toast.makeText(getApplicationContext(), "블루투스가 이미 활성화 되어 있습니다.", Toast.LENGTH_LONG).show();
                mTvBluetoothStatus.setText("활성화");
            }
            else {
                Toast.makeText(getApplicationContext(), "블루투스가 활성화 되어 있지 않습니다.", Toast.LENGTH_LONG).show();
                Intent intentBluetoothEnable = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(intentBluetoothEnable, BT_REQUEST_ENABLE);
            }
        }
    }
    void bluetoothOff() {
        if (mBluetoothAdapter.isEnabled()) {
            mBluetoothAdapter.disable();
            Toast.makeText(getApplicationContext(), "블루투스가 비활성화 되었습니다.", Toast.LENGTH_SHORT).show();
            mTvBluetoothStatus.setText("비활성화");
        }
        else {
            Toast.makeText(getApplicationContext(), "블루투스가 이미 비활성화 되어 있습니다.", Toast.LENGTH_SHORT).show();
        }
    }
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {
            case BT_REQUEST_ENABLE:
                if (resultCode == RESULT_OK) { // 블루투스 활성화를 확인을 클릭하였다면
                    Toast.makeText(getApplicationContext(), "블루투스 활성화", Toast.LENGTH_LONG).show();
                    mTvBluetoothStatus.setText("활성화");
                } else if (resultCode == RESULT_CANCELED) { // 블루투스 활성화를 취소를 클릭하였다면
                    Toast.makeText(getApplicationContext(), "취소", Toast.LENGTH_LONG).show();
                    mTvBluetoothStatus.setText("비활성화");
                }
                break;
        }
        super.onActivityResult(requestCode, resultCode, data);
    }
    void listPairedDevices() {
        if (mBluetoothAdapter.isEnabled()) {
            mPairedDevices = mBluetoothAdapter.getBondedDevices();

            if (mPairedDevices.size() > 0) {
                AlertDialog.Builder builder = new AlertDialog.Builder(this);
                builder.setTitle("장치 선택");

                mListPairedDevices = new ArrayList<String>();
                for (BluetoothDevice device : mPairedDevices) {
                    mListPairedDevices.add(device.getName());
                    //mListPairedDevices.add(device.getName() + "\n" + device.getAddress());
                }
                final CharSequence[] items = mListPairedDevices.toArray(new CharSequence[mListPairedDevices.size()]);
                mListPairedDevices.toArray(new CharSequence[mListPairedDevices.size()]);

                builder.setItems(items, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int item) {
                        connectSelectedDevice(items[item].toString());
                    }
                });
                AlertDialog alert = builder.create();
                alert.show();
            } else {
                Toast.makeText(getApplicationContext(), "페어링된 장치가 없습니다.", Toast.LENGTH_LONG).show();
            }
        }
        else {
            Toast.makeText(getApplicationContext(), "블루투스가 비활성화 되어 있습니다.", Toast.LENGTH_SHORT).show();
        }
    }
    void connectSelectedDevice(String selectedDeviceName) {
        for(BluetoothDevice tempDevice : mPairedDevices) {
            if (selectedDeviceName.equals(tempDevice.getName())) {
                mBluetoothDevice = tempDevice;
                break;
            }
        }
        try {
            mBluetoothSocket = mBluetoothDevice.createRfcommSocketToServiceRecord(BT_UUID);
            mBluetoothSocket.connect();
            mThreadConnectedBluetooth = new ConnectedBluetoothThread(mBluetoothSocket);
            mThreadConnectedBluetooth.start();
            mBluetoothHandler.obtainMessage(BT_CONNECTING_STATUS, 1, -1).sendToTarget();
        } catch (IOException e) {
            Toast.makeText(getApplicationContext(), "블루투스 연결 중 오류가 발생했습니다.", Toast.LENGTH_LONG).show();
        }
    }
    
```

# 데모 영상 시연
![ddd](/img/c1.jpg)
![ddd](/img/c2.jpg)
![ddd](/img/d1.jpg)
![ddd](/img/d2.jpg)

# 알려진 이슈
## 1. 시리얼 통신으로 인한 딜레이의 존재
- 웹캠과 아두이노 간의 시리얼 통신으로 인해서 동작에 전체적으로 딜레이가 걸리게 되고 딜레이 동안 IR 센서를 통한 라인트레이서가 정상적으로 동작하지 않음.
- ※ 카메라와 아두이노를 시리얼 통신으로 동작하는 것이 아닌 핀을 통해 직접적으로 데이터를 주고 받는 것으로 개선 가능.
- ※ Serial.end()와 Serial.begin()의 조건을 추가해서 원할 때만 시리얼 통신을 하도록 코드를 변경하는 것으로 개선 가능.

## 2. 웹캠의 유선 연결 문제
- 웹캠이 유선을 통해서 데이터를 전송하므로 선 연결의 문제가 있었음.
- 직접 적으로 본체에 웹캠 USB 선을 꽂았을 때 openCV가 비디오를 인식하는 확률은 대략 90%, 연장선을 이용했을 때 인식하는 확률은 대략 10%.
- 동작 도중에도 라인트레이서의 모터가 주는 진동으로 인해서 연결이 해제되는 경우가 많았음.
- ※ 노트북 내장캠, 휴대폰 카메라 등 안정적인 카메라와 블루투스 연결을 통해 데이터를 주고 받아야 함.

## 3. 카메라의 속도와 라인트레이서 구동 속도의 차이로 인한 문제
- DRGO 웹캠은 초당 30프레임을 전송하는데 라인 트레이서의 구동속도가 너무 빠른 나머지 라인이 카메라 인식 범위 밖으로 빠져나가는 현상이 발생.
- ※ 하드웨어적인 성능 개선이 필요 ex) 초당 60프레임 카메라 or 더 넓은 화면을 촬영하는 카메라 or PWM 조절을 통해서 더 부드러운 동작이 가능한 라인트레이서.

## 4. 라인 인식의 문제
- 현재 라인트레이서의 IR 센서는 검은색 절연테이프를 읽도록 설정되어 있으나 바닥의 검은색 자국도 라인으로 인식하는 문제가 있음.
- ※ 1. IR 센서의 가변저항기를 조절하여 센서의 감도를 조절할 수 있다고 함 → 실패
- ※ 2. 자국 패턴과 라인 패턴을 인식시켜서 라인만 인지하도록 해야 함. 
- ※ 3. 라인 조건 변경 : 라인 색을 다르게 하던가 혹은 바탕을 하얀색으로 수정해야 함.
