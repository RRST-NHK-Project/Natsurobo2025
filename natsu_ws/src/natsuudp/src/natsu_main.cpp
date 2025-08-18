/*
RRST-NHK-Project 2010 夏ロボ
機構制御
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"


#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1)

// 各機構の速さの指定(%)
int speed_lift_up = 60;
int speed_lift_down = -60;



//ラッチ用の文字設定
bool SERVOMODE = false;
bool VERTICALMODE = false;
int SHOOTMODE = 0;

std::vector<int16_t> data(19, 0); // マイコンに送信される配列"data"

/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ
| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
| data[1] | MD1 | -100 ~ 100 |
| data[2] | MD2 | -100 ~ 100 |
| data[3] | MD3 | -100 ~ 100 |
| data[4] | MD4 | -100 ~ 100 |
| data[5] | MD5 | -100 ~ 100 |
| data[6] | MD6 | -100 ~ 100 |
| data[7] | Servo1 | 0 ~ 80 |
| data[8] | Servo2 | 0 ~ 80 |
| data[9] | Servo3 | 0 ~ 80 |
| data[10] | Servo4 | 0 ~ 80 |
| data[11] | TR1 | 0 or 1|  //VGOAL
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|  //ポンプ１
| data[14] | TR4 | 0 or 1|   //ポンプ２
| data[15] | TR5 | 0 or 1|  //シリンダ
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|

*/

//初期化
class Init{
public:
    static void init(UDP &udp){
        for(int i = 1; i <= 18; i++){
            data[i] = 0;
        }
        udp.send(data);
        std::cout << "初期化完了" << std::endl;
    }
};





// リフト機構のシーケンスを格納するクラス
class Lift_Action{
public: 
    static void lift_up_action(UDP &udp) {
            std::cout << "リフト準備開始" << std::endl;
            data[4] = speed_lift_up = 60;
            std::this_thread::sleep_for(std::chrono::milliseconds(20000));
            udp.send(data);
            std::cout << "リフト上昇中..." << std::endl;
            data[4] = 0;
            udp.send(data);
            std::cout << "リフト上昇完了." << std::endl;
    }

    static void lift_down_action(UDP &udp) {
            std::cout << "リフト準備開始" << std::endl;
            data[4] = speed_lift_down = -60;
            std::this_thread::sleep_for(std::chrono::milliseconds(20000));
            udp.send(data);
            std::cout << "リフト下降中..." << std::endl;
            data[4] = 0;
            udp.send(data);
            std::cout << "リフト下降完了." << std::endl;
    }
};

//フォークの機構のシーケンスを格納
class Folk_Action{

       /* フォーク機構割当て
        //servo
        data[7] = 0;
        data[8] = 0;　//垂直
        //ポンプ
        data[13] = 0;
        data[14] = 0;
        */
   
    
public:
    //サーボの状態管理
    static bool state_servo_0;
    static bool state_servo_ver_0;
    
    // 段ボール(柱)回収シーケンス
    // サーボ０°&ポンプ０　<-> サーボ90°&ポンプ1 
    static void init_folk_action(UDP &udp) {
            std::cout << "準備中" << std::endl;
            data[1] = 0;
            data[2] = 0;
            data[7] = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            udp.send(data);
            std::cout << "ポンプOFF" << std::endl;
    }

    static void folk_action(UDP &udp) {
            std::cout << "準備中" << std::endl;
            data[1] = 50;
            data[2] = 50;
            data[7] = 90;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            udp.send(data);
            std::cout << "ポンプON" << std::endl;
            std::cout << "完了." << std::endl;
    }
   
    // リフト機構が垂直の場合の柱回収用制御
    // サーボ０°&ポンプ０　<-> サーボ90°&ポンプ1 
   static void init_vertical_folk_action(UDP &udp) {
           std::cout << "準備中" << std::endl;
            data[1] = 0;
            data[2] = 0;
            data[8] = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            udp.send(data);
            std::cout << "垂直方向。ポンプOFF" << std::endl;
    }

    static void vertical_folk_action(UDP &udp) {
           std::cout << "準備中" << std::endl;
            data[1] = 50;
            data[2] = 50;
            data[8] = 90;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            udp.send(data);
            std::cout << "垂直方向。ポンプON" << std::endl;

    }

};



class Ball_Action{
public:
    //ボール(餅)回収シーケンス
    static void init_ball_action(UDP &udp){
            std::cout << "準備中" << std::endl;
            data[9] =0;
            udp.send(data);
            std::cout << "サーボ０" << std::endl;
    }

    static void ball_action(UDP &udp){
            std::cout << "餅回収..." << std::endl;
            data[9] = 90;
            udp.send(data);
            std::cout << "サーボ９０" << std::endl;
    }
 
};

class Shoot_Yaw_Action{
public:
    //ボール(餅)回収シーケンス
    static  void init_shoot_yaw_action(UDP &udp){
            std::cout << "準備中" << std::endl;
            data[10] =0;
            udp.send(data);
            std::cout << "サーボ０" << std::endl;
    }

    static void shoot_yaw_action(UDP &udp){
            std::cout << "餅回収..." << std::endl;
            data[10] = 90;
            udp.send(data);
            std::cout << "サーボ９０" << std::endl;
    }
 
};




class Vgoal{    
public:
    static void vgoal_action(UDP &udp) {
    data[11] = 1;
    udp.send(data);
     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Vゴール!" << std::endl;
     data[11] = 0;
    udp.send(data);
    std::cout << "######" << std::endl;
    }

};

   // テスト用！！実機で実行するな！！！！
    /*
    static void tester(UDP &udp) {
        int tester_time = 150;
        while (1) {
            for (int i = 11; i <= 18; ++i) {
                data[i] = 1; 
                udp.send(data);
                std::this_thread::sleep_for(std::chrono::milliseconds(tester_time));
            }
            for (int i = 11; i <= 18; ++i) {
                data[i] = 0;
                udp.send(data);
                std::this_thread::sleep_for(std::chrono::milliseconds(tester_time));
            }
        }
    }
        */



class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk10_natsu"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NATSUROBO2010 initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
    }

private:
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        //  float LS_X = -1 * msg->axes[0];
        //  float LS_Y = msg->axes[1];
        //  float RS_X = -1 * msg->axes[3];
        //  float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        //float L2 = (-1 * msg->axes[2] + 1) / 2;
        //float R2 = (-1 * msg->axes[5] + 1) / 2;

        bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        static bool last_circle = false; // 前回の状態を保持する static 変数
        static bool last_triangle= false;
        static bool last_square= false;

        // ラッチstatic 変数（初期状態は OFF とする）
        static bool circle_latch = false;
        static bool triangle_latch = false;
        static int square_mode = 0;

        data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)

        if (PS) {
            std::fill(data.begin(), data.end(), 0);          // 配列をゼロで埋める
            for (int attempt = 0; attempt < 10; attempt++) { // 10回試行
                udp_.send(data);                             // データ送信
                std::cout << "緊急停止！ 試行" << attempt + 1
                          << std::endl; // 試行回数を表示
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(100)); // 100msの遅延
            }
            rclcpp::shutdown();
        }
        //ラッチ(ボタンで変更)
        if (CIRCLE && !last_circle) {
            circle_latch = !circle_latch;
        }
        if (TRIANGLE && !last_triangle) {
            triangle_latch = !triangle_latch;
        }
        if (SQUARE && !last_square) {
            square_mode = (square_mode + 1) % 3;
        }

        //ラッチのボタンとモードの指定
        last_circle = CIRCLE;
        SERVOMODE = circle_latch;
        last_triangle = TRIANGLE;
        VERTICALMODE = triangle_latch;
        last_square = SQUARE;
        SHOOTMODE = square_mode;

    
        //機構を初期状態にする
        if (CROSS) {
                Init::init(udp_);
        }

        //シュート機構
        // ボタンを一回押すごとに速度変更
        if (SHOOTMODE == 0){
            Shoot_Action::shot_action(udp_);
        }

        if (SHOOTMODE == 1){
            Shoot_Action::shoot_action(udp_);
        }

        if (SHOOTMODE == 2) {
            Shoot_Action::longshoot_action(udp_);
        }

  
        //リフト機構
        if (UP) {
            Lift_Action::lift_up_action(udp_);
        }

        if (DOWN) {
            Lift_Action::lift_down_action(udp_);
        }

        //ボール回収機構
        if(R1) {
            Ball_Action::init_ball_action(udp_);
        }

        if(L1) {
            Ball_Action::ball_action(udp_);
        }

        //
        if(LEFT) {
            Shoot_Yaw_Action::init_shoot_yaw_action(udp_);
        }

        if(RIGHT) {
            Shoot_Yaw_Action::shoot_yaw_action(udp_);
        }

        if(SHARE){
            Vgoal::vgoal_action(udp_);
        }

        // リフトのフォーク機構
        //　サーボ０°＋ポンプ０　-> サーボ90°＋ポンプ1 -> サーボ0°＋ポンプ0
    
        if (SERVOMODE == 1) {
                    Folk_Action::folk_action(udp_);
                }
        if (SERVOMODE == 0) {
                    Folk_Action::init_folk_action(udp_);
                }
            
        

        ////ポンプ機構が垂直の場合
        if (VERTICALMODE == 1) {
                    Folk_Action::vertical_folk_action(udp_);
            }
        if (VERTICALMODE == 0){
                    Folk_Action::init_vertical_folk_action(udp_);
            }
        

        // if (OPTION) {
        //     Ball_Action::tester(udp_);
        // } 
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        udp_.send(data);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    UDP udp_;
};


class Params_Listener : public rclcpp::Node {
public:
    Params_Listener() : Node("natsu_pr_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "parameter_array", 10,
            std::bind(&Params_Listener::params_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),

                    "NATSUROBO2010 Parameter Listener initialized");
    }

private:
    void params_listener_callback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        speed_lift_up = msg->data[0];
        speed_lift_down  = msg->data[1];
        speed_shot = msg->data[2];
        speed_shoot = msg->data[3];      
        speed_longshoot = msg->data[4];
        std::cout << speed_lift_up;
        std::cout << speed_lift_down;
        std::cout << speed_shot;
        std::cout << speed_shoot;
        std::cout << speed_longshoot << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet RRST NATSUROBO";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_NATSU, PORT_NATSU);
    auto params_listener = std::make_shared<Params_Listener>();
    exec.add_node(ps4_listener);
    exec.add_node(params_listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
