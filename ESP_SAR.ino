
#define LED 2

#define SERIAL_BAUD_RATE 115200
#define PACKET_START_BYTE 0xAA
#define PACKET_STOP_BYTE 0x55
#define PACKET_HEADER_BYTE 0xBB
#define PACKET_CRC_SIZE 1 // CRC is now one byte (CRC-8)
bool flag = false;
struct Received_ControlData {
  bool grasp_command;
  int8_t stroke_min;
  int8_t stroke_max;
  int8_t set_max_contact_force;
  int8_t set_velocity;
  int8_t set_axial_force_threshold;
  int8_t set_lateral_force_threshold;
};

typedef struct {
  bool grasp_command;
  uint8_t set_velocity;
  uint8_t stroke_min;
  uint8_t stroke_max;
  uint8_t set_max_contact_force;
  uint8_t set_axial_force_threshold;
  uint8_t set_lateral_force_threshold;
  bool grasp_status;
  bool axial_force_threshold_status;
  bool lateral_force_threshold_status;
  uint8_t current_stroke_length;
  uint8_t current_sensor;
  uint8_t joint_torque_estimation;
  uint8_t axial_force_feedback;
  uint8_t lateral_force_feedback;
} ControlData;

typedef struct {
  uint8_t start;
  uint8_t header;
  uint8_t data[sizeof(ControlData)];
  uint8_t crc[PACKET_CRC_SIZE];
  uint8_t stop;
} CustomPacket;

// CRC-8 calculation function (8-bit CRC)
uint8_t calculateCRC8(const uint8_t *data, size_t length) {
  uint8_t crc = 0x00; // Initial value
  const uint8_t polynomial = 0x8C; // CRC-8 polynomial

  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ polynomial;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}




CustomPacket packet;

ControlData controlData;

void packData(const ControlData *data, CustomPacket *packet) {
  packet->start = PACKET_START_BYTE;
  packet->header = PACKET_HEADER_BYTE;
  memcpy(packet->data, data, sizeof(ControlData));
  // Calculate CRC for data
  uint8_t crc = calculateCRC8(packet->data, sizeof(ControlData));
  packet->crc[0] = crc; // Store CRC directly as one byte
  packet->stop = PACKET_STOP_BYTE;
}

void sendPacket(const CustomPacket *packet) {
  Serial2.write(packet->start);
  Serial2.write(packet->header);
  Serial2.write(packet->data, sizeof(packet->data));
  Serial2.write(packet->crc, sizeof(packet->crc));
  Serial2.write(packet->stop);
}

void setup() {
  Serial.begin(500000);
  Serial2.begin(256000);
  pinMode(LED, OUTPUT);
}
int i = 0, cmd = 0;
unsigned long pm = 0;
Received_ControlData control_data;
void loop() {
  if (Serial2.available() > 0) {
    {
      cmd = Serial2.read();
    } if (cmd == 128) {
      controlData.grasp_command = !controlData.grasp_command;
      controlData.set_velocity = controlData.set_velocity + 10;
      controlData.stroke_min = controlData.stroke_min + 10;
      controlData.stroke_max = controlData.stroke_max + 10;
      controlData.set_max_contact_force = controlData.set_max_contact_force + 10;
      controlData.set_axial_force_threshold = controlData.set_axial_force_threshold + 10;
      controlData.set_lateral_force_threshold = controlData.set_lateral_force_threshold + 10;
      controlData.grasp_status = !controlData.grasp_status;
      controlData.axial_force_threshold_status = !controlData.axial_force_threshold_status;
      controlData.lateral_force_threshold_status = !controlData.lateral_force_threshold_status;
      controlData.current_stroke_length = controlData.current_stroke_length + 10;
      controlData.current_sensor = controlData.current_sensor + 10;
      controlData.joint_torque_estimation = controlData.joint_torque_estimation + 10;
      controlData.axial_force_feedback = controlData.axial_force_feedback + 10;
      controlData.lateral_force_feedback = controlData.lateral_force_feedback + 10;

      //      Serial.print(cmd); Serial.println();
      packData(&controlData, &packet);
      sendPacket(&packet);
    } if (cmd == 129) {
      Serial2.readBytes((char*)&control_data, sizeof(Received_ControlData));
      Serial.print("Grasp Command: ");
      Serial.println(control_data.grasp_command);
      Serial.print("Stroke Min: ");
      Serial.println(control_data.stroke_min);
      Serial.print("Stroke Max: ");
      Serial.println(control_data.stroke_max);
      Serial.print("set_max_contact_force: ");
      Serial.println(control_data.set_max_contact_force);
      Serial.print("set_velocity: ");
      Serial.println(control_data.set_velocity);
      Serial.print("set_axial_force_threshold: ");
      Serial.println(control_data.set_axial_force_threshold);
      Serial.print("set_lateral_force_threshold: ");
      Serial.println(control_data.set_lateral_force_threshold);
      digitalWrite(2, !digitalRead(2));
    }

  }
}
