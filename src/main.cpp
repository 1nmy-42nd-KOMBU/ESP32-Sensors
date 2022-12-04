TaskHandle_t thp[1];//マルチスレッドのタスクハンドル格納用
long a = 0;

void setup() {
  Serial.begin(115200);
  xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 3, &thp[0], 0); 
}

void loop() {
  Serial.println(a);//メインで実行するプログラム
  delay(100);
}

void Core0a(void *args) {
  while (1) {
    delay(1);//サブで実行するプログラムを書く
    a++;
  }
}