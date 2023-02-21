#include <DynamixelSDK.h>

#define PROTOCOL_VERSION      1.0
#define BAUDRATE              1000000
#define DEVICENAME            "3"

dynamixel::PacketHandler *packetHandler;
dynamixel::PortHandler *portHandler;


int count = -1;
int third = -1;

void setup() {
  Serial.begin(115200);
  Serial2.begin(57600);
  //while(!Serial);
  Serial.println("Start..");
  
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (portHandler->openPort()){
    Serial.print("Succeeded to open the port!\n");
  }else{
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  if (portHandler->setBaudRate(BAUDRATE)){
    Serial.print("Succeeded to change the baudrate!\n");
  }else{
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  //その他初期化処理はここから記入
  //setTorque(14,1);
  //setTorque(11,1);
}

void loop() {
  //ループ処理をここに記入
  char key;
  if(Serial2.available()){
    //key = (char)Serial2.read();
    String message = (String)Serial2.readString();
    key = message.charAt(0);
    String count_text = message.substring(1,message.length());
    count = count_text.toInt();
    }

    if(count >= 0){
      int t = millis();
      int newthird = (t/100)%10;
      if(third != newthird){
        if(count == 0)  key = 'e';
        third = newthird;
        count--;
      } 
    }

    if(key == 'e'){
      setWheelSpeed(11, 0, 0);
      setWheelSpeed(14, 0, 0);
    }
    else if(key == 'w'){
      setWheelSpeed(11, 256, 1);
      setWheelSpeed(14, 256, 0);
    }
    else if(key == 's'){
      setWheelSpeed(11, 256, 0);
      setWheelSpeed(14, 256, 1);
    }
    else if(key == 'a'){
      setWheelSpeed(11, 128, 1);
      setWheelSpeed(14, 128, 1);
    }
    else if(key == 'd'){
      setWheelSpeed(11, 128, 0);
      setWheelSpeed(14, 128, 0);
    }
    else if(key == 'z'){
      setJointSpeed(12, 1023);
      setPosition(12, 512);
      delay(1000);
      setPosition(12, 150);
      delay(500);
      setPosition(12, 512);
    }
}

//DXLに関する5つの関数を用意
//setTorque(int dxl_id, int torque)
//setJointSpeed(int dxl_id, int speed)
//setWheelSpeed(int dxl_id, int speed, int direction)
//setPosition(int dxl_id, int position)
//setPositionWithDelay(int dxl_id, int position)
//詳細はそれぞれの関数の上に記載

//トルクのオン/オフを切り替えます
//dxl_id: DXLのIDを指定
//torque: 0=オフ 1=オン
void setTorque(int dxl_id, int torque){
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, torque, &dxl_error);
  
  if(dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  else if(dxl_error != 0) packetHandler->getRxPacketError(dxl_error);
  else  Serial.print("Dynamixel has been successfully connected \n");
}

//ホイールモードDXLの回転速度を変更します
//dxl_id: DXLのIDを指定
//speed: 0~1023でスピードを指定
//direction: 0=反時計回り 1=時計回り
void setWheelSpeed(int dxl_id, int speed, int direction){
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, 32, speed+(1024*direction), &dxl_error);
  
  if(dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  else if(dxl_error != 0) packetHandler->getRxPacketError(dxl_error);
}

//ジョイントモードDXLの回転速度を変更します
//dxl_id: DXLのIDを指定
//speed: 0~1023でスピードを指定
void setJointSpeed(int dxl_id, int speed){
  setWheelSpeed(dxl_id, speed, 0);
}

//ジョイントモードDXLの回転位置を変更します
//dxl_id: DXLのIDを指定
//position: 0~1023で回転位置を指定
void setPosition(int dxl_id, int position){
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, 30, position, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  else if(dxl_error != 0) packetHandler->getRxPacketError(dxl_error);
}

//ジョイントモードDXLの回転位置を変更します
//回転が終わるまで次の命令が実行されません
//dxl_id: DXLのIDを指定
//position: 0~1023で回転位置を指定
void setPositionWithDelay(int dxl_id, int position){
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  uint16_t dxl_present_position = 0;
  
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, 30, position, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  else if(dxl_error != 0) packetHandler->getRxPacketError(dxl_error);

  do{
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, (uint16_t*)&dxl_present_position, &dxl_error);
  }while((abs(position - dxl_present_position) > 10));
}
