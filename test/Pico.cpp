//
// 3桁の7セグメントLEDを光らせるプログラム
//
int num_3digits = 100;

void setup(){
    //1～10番ピン　デジタル出力へセット
    for (int i=1; i<=10; i++){
        pinMode(i,OUTPUT);
    }
}
//LEDレイアウトを定義
boolean Num_Array[11][7]={
//pin1,2,3,4,5,6,7
    {1,1,1,1,0,1,1}, //0
    {0,1,0,0,0,0,1}, //1
    {0,0,1,1,1,1,1}, //2
    {0,1,1,0,1,1,1}, //3
    {1,1,0,0,1,0,1}, //4
    {1,1,1,0,1,1,0}, //5
    {1,1,1,1,1,1,0}, //6
    {1,1,0,0,0,1,1}, //7
    {1,1,1,1,1,1,1}, //8
    {1,1,1,0,1,1,1}, //9
    {0,0,0,0,1,0,0} // empty
};

//LED表示関数を定義
void NumPrint(int number){
    for (int i=1; i<=7; ++){
        digitalWrite(i,Num_Array[number][i-1]);
    }
}

void loop(){
    // 4桁以上だったら999に抑える
    if num_3digits >= 1000{
        num_3digits = 999;
    }

    // 1桁目
    digitalWrite(10,LOW);
    digitalWrite(9.HIGH);
    digitalwrite(8,HIGH);
    NumPrint(num_3digits / 100);

    // 2桁目
    digitalWrite(10,HIGH);
    digitalWrite(9.LOW);
    digitalwrite(8,HIGH);
    NumPrint(num_3digits % 100 / 10);

    // 3桁目
    digitalWrite(10,HIGH);
    digitalWrite(9.HIGH);
    digitalwrite(8,LOW);
    NumPrint(num_3digits % 10);
}