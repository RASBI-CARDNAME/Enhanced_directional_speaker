# 🔊 STM32 기반 초지향성 스피커 (Ultrasonic Directional Speaker)

## 1. 프로젝트 개요 (Overview)
초음파의 직진성과 공기의 비선형성(Non-linearity)을 이용하여 특정 영역에만 소리를 전달하는 **초지향성 스피커(Parametric Speaker)** 프로젝트입니다.
오픈소스 프로젝트(Arduino 기반)를 분석하여 **STM32 HAL 라이브러리로 직접 포팅**하였으며, **하드웨어 회로 개선(LC 공진 및 노이즈 필터링)을 통해 음질과 출력을 대폭 향상**시켰습니다.

*   **원본 레포지토리:** [gururise/directional_speaker](https://github.com/gururise/directional_speaker)
*   **원본 레포 작동 영상:** [YouTube](https://www.youtube.com/watch?v=9hD5FPVSsV0)
*   **이 레포 작동 영상 및 측정치:** [Naver Blog](https://blog.naver.com/hiho0718/224090581630)
  
*   **개선 사항:**
    1.  **SW:** Arduino 코드를 STM32CubeIDE(HAL) 환경으로 포팅 및 PWM 로직 최적화
    2.  **HW:** TC4427 드라이버 안정화 및 LC 공진 회로 추가를 통한 출력 효율 증대

---

## 2. 기술 스택 (Tech Stack)
*   **MCU:** STM32F103C8T6 (WeAct BluePill Plus)
*   **IDE/Language:** STM32CubeIDE, C (HAL Library)
*   **Hardware:** KiCad (Schematic), TC4427 (MOSFET Driver), Ultrasonic Transducers (40kHz)
*   **Theory:** Amplitude Modulation (AM), PWM Carrier Generation

---

## 3. 핵심 개선 사항 (Key Improvements)

### 🛠️ 하드웨어 (Hardware)
단순한 드라이버 구동을 넘어, 물리적 특성을 고려한 회로 튜닝을 진행했습니다.
*   **LC 공진 회로(Resonant Circuit) 구성:**
    *   초음파 트랜스듀서(Transducer)가 가진 고유 커패시턴스 성분을 고려하여, 인덕터(Inductor)를 추가해 LC 공진을 유도했습니다.
    *   이를 통해 전력 효율을 높이고 스피커 출력을 극대화했습니다. (트랜스듀서 개수에 맞춰 인덕턴스 값 튜닝)
*   **전원 안정화:**
    *   TC4427 MOSFET 드라이버에 **Decoupling Capacitor**를 추가하여 고속 스위칭 시 발생하는 전압 강하와 노이즈를 억제, 음질을 개선했습니다.

### 💻 소프트웨어 (Firmware)
*   **HAL 라이브러리 포팅:**
    *   기존 Arduino의 레지스터 직접 제어 방식을 분석하여, STM32 HAL 및 LL 드라이버 구조에 맞게 재설계했습니다.
*   **PWM 신호 무결성 확보:**
    *   40kHz Carrier 주파수를 생성하고, ADC로 입력받은 오디오 신호에 따라 Duty Cycle을 실시간으로 변조합니다.

---

## 4. 트러블 슈팅 (Troubleshooting) 🚀핵심

### Q1. 볼륨 증가 시 음질 저하 및 클리핑 현상
*   **원인:** PWM 듀티 사이클이 0%나 100%에 근접할 경우, 파형이 DC(직류)에 가까워지며 트랜스듀서의 응답성이 떨어지고 신호가 찌그러짐.
*   **해결:** **Software Limiter 구현.**
    *   PWM Duty Cycle의 범위를 **33% ~ 88% (또는 유효 구간)** 내로 제한하는 로직을 추가하여, 변조 한계를 방지하고 음량 헤드룸(Headroom)을 확보함. 임의로 듀티 사이클 범위를 변경시 음질 저하가 발생 할 수 있으니 주의.

### Q2. 40kHz 트랜스듀서 구동 효율 문제
*   **원인:** 트랜스듀서는 용량성(Capacitive) 부하이기 때문에 단순 구동 시 증폭 단 전류 소모가 크고 심한 열이 발생.
*   **해결:** 직렬 인덕터를 추가하여 회로가 **특정 주파수(40kHz)에서 공진**하도록 설계, 임피던스 매칭을 통해 구동 효율 개선.

---

## 5. 하드웨어 구성 (Components)
*   **MCU Board:** WeAct BluePill Plus (STM32F103C8T6)
*   **Driver IC:** TC4427 (High-Speed MOSFET Driver)
*   **Transducer:** V40AN16T (40kHz) x 42 units array
*   **Pre-Amp:** LM358 (Audio Input Bias & Amplification)

## 트랜스듀서
40kHz에서 작동하는 V40AN16T 트랜스듀서를 42개 사용했습니다.
이 주파수와 일치하는 트랜스듀서를 사용해야 합니다.

## 다른 연산 증폭기나 MOSFET 드라이버를 사용할 수 있나요?
상관 없습니다. 부품을 자유롭게 업그레이드할 수 있습니다.

예를 들어, 더 나은 성능을 위해 MOSFET 드라이버 + 개별 MOSFET 구성을 사용할 수 있습니다.
아시다시피, PWM 신호를 증폭하기 위해 MOSFET 드라이버만 사용하는 것은 이상적이지 않지만, 이 작품에서는 잘 작동합니다.


## 데드타임 설정이 되어있나요?
회로도를 따라 구현한다면 데드타임이 필요하지 않지만, 다른 증폭 방식을 사용할때는 데드타임을 설정해주어야 합니다.

---

## 📜 License
*   Original Code: GPL 2.0 License (by gururise)
*   Modified Code: GPL 2.0 License
