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

    # 결과를 처리합니다.
    cx_list = []
    cy_list = []
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

            # 객체의 중심점(cx, cy) 계산
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            cx_list.append(cx)
            cy_list.append(cy)

            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    # person 객체가 감지되지 않은 경우
    if not person_detected:
        print('D1')
        ser.write(b'D')  # D 신호를 보냅니다.
    # person 객체가 감지된 경우
    else:
        # 여러 명의 person 객체의 중심점 평균 계산
        if len(cx_list) > 0 and len(cy_list) > 0:
            avg_cx = int(np.mean(cx_list))
            avg_cy = int(np.mean(cy_list))

            # 중심점 정보 시리얼 통신으로 전송하기
            if avg_cx > 160 and avg_cx < 480:
                print('T')
                ser.write(b'T')
            else:
                print('D')
                ser.write(b'D')

            cv2.circle(frame, (avg_cx, avg_cy), 5, (255, 0, 0), -1)

    # person_detected 변수를 False로 초기화하여 다음 프레임에서 다시 감지할 수 있도록 합니다.
    person_detected = False

    cv2.imshow('YOLOv5', frame)  # 화면에 프레임을 출력합니다.

    height, width, channels = frame.shape

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

    # 컨투어 중 가장 큰 영역 찾기
    max_area = 0
    max_contour = None

    min_contour_area = 15000

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
            if cx < 140:
                print('L')
                ser.write(b'L')
            elif cx > 180:
                print('R')
                ser.write(b'R')
            else:
                print('F')
                ser.write(b'F')

    elif max_contour is None:
        print("D2")
        ser.write(b'D')

    # 프레임 보여주기
    cv2.imshow("Line Tracer", thresh)

    # 'q' 키를 눌러 종료합니다.
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
