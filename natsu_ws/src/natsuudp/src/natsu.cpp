/*
RRST-NHK-Project 2025 夏ロボ
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
int speed_lift_up = 30;
int speed_lift_down = -20;


//射出機構の速さ
int speed_shot = 25;
int speed_shoot = 35;
int speed_longshoot = 45;

std::vector<int16_t> data(37, 0); // マイコンに送信される配列"data"
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
| data[7] | Servo1 | 0 ~ 270 |
| data[8] | Servo2 | 0 ~ 270 |
| data[9] | Servo3 | 0 ~ 270 |
| data[10] | Servo4 | 0 ~ 270 |
| data[11] | TR1 | 0 or 1|
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|
| data[14] | TR4 | 0 or 1|
| data[15] | TR5 | 0 or 1|
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|
//mbed2
| data[19] | MD1 | -100 ~ 100 |
| data[20] | MD2 | -100 ~ 100 | 
| data[21] | MD3 | -100 ~ 100 | 
| data[22] | MD4 | -100 ~ 100 | 
| data[23] | MD5 | -100 ~ 100 |
| data[24] | MD6 | -100 ~ 100 |
| data[25] | Servo1 | 0 ~ 270 |
| data[26] | Servo2 | 0 ~ 270 |
| data[27] | Servo3 | 0 ~ 270 |
| data[28] | Servo4 | 0 ~ 270 |
| data[29] | TR1 | 0 or 1|　
| data[30] | TR2 | 0 or 1|
| data[31] | TR3 | 0 or 1|  //ポンプ１
| data[32] | TR4 | 0 or 1|  //ポンプ２
| data[33] | TR5 | 0 or 1|  //シリンダ
| data[34] | TR6 | 0 or 1|
| data[35] | TR7 | 0 or 1|
| data[36] | TR8 | 0 or 1|
*/

// 各機構のシーケンスを格納するクラス
class Action {
public: 
    
    /*射出機構*/
    // 射出シーケンス(１段階目)
    static void shot_action(UDP &udp) {
        data[19] = speed_shot;
        data[34] = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        udp.send(data);
        std::cout << "ショット" << std::endl;
        data[19] = 0;
        data[34] = 0;
        udp.send(data);
        std::cout << "完了" << std::endl;
    }
    // 射出シーケンス(2段階目)
    static void shoot_action(UDP &udp) {
        data[19] = speed_shoot;
        data[35] = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        udp.send(data);
        std::cout << "シュート" << std::endl;
        data[19] = 0;
        data[35] = 0;
        udp.send(data);
        std::cout << "完了" << std::endl;
    }
    // 射出シーケンス(3段階目)
    static void longshoot_action(UDP &udp) {
        data[19] = speed_longshoot;
        data[36] = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        udp.send(data);
        std::cout << "ロングシュート" << std::endl;
        data[19] = 0;
        data[36] = 0;
        udp.send(data); 
        std::cout << "完了" << std::endl;
    }


    //ボール(餅)回収シーケンス
    static void collect_ball_action(UDP &udp) {
        data[25] = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        udp.send(data);
        std::cout << "初期位置に戻る" << std::endl;
        data[25] = 45;     
        udp.send(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "ボール回収準備中..." << std::endl;
        data[25] = 90;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "ボール回収中..." << std::endl;
        udp.send(data);
        data[25] = 0;
        std::cout << "完了." << std::endl;
        udp.send(data);
    }

    //段ボール(柱)シーケンス

       /* 必要がなければコメントアウト
        //servo
        data[26] = 0;
        data[27] = 0;
        //MD
        data[23] = 0;
        //ポンプ&シリンダ
        data[31] = 0;
        data[32] = 0;
        data[33] = 0;
        */

    //リフトの状態管理
    static bool state_folk_up;
    static bool state_folk_middle;
    static bool state_folk_down;
    
    //段ボール(柱)回収シーケンス
    static void ready_collect_box_action(UDP &udp) {
            data[26] = 0;
            data[27] = 0;
            udp.send(data);
            std::cout << "上棟準備開始" << std::endl;
            std::cout << "上棟準備中..." << std::endl;
            data[26] = 90;
            data[27] = 90;
            udp.send(data);
            std::cout << " ..." << std::endl;
            std::cout << "上棟準備完了" << std::endl;
            state_folk_up = true;
    }

    static void collect_box_action(UDP &udp) {
            data[31] = 1;
            data[32] = 1;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            udp.send(data);
            std::cout << "柱回収中" << std::endl;
            data[31] = 0;
            data[32] = 0;
            udp.send(data);
            state_folk_middle = true;
            std::cout << "完了." << std::endl;
    }
   
   static void end_collect_box_action(UDP &udp) {
            state_folk_up = true;
            std::cout << "上棟開始" << std::endl;
            data[26] = 0;
            data[27] = 0;
            udp.send(data);
            std::cout << " 上棟完了." << std::endl;
            state_folk_down = true;
            std::cout << "柱回収シーケンス完了" << std::endl;
    }

    
    static void lift_up_action(UDP &udp) {
            state_folk_up = true;
            data[33] = 1;
            udp.send(data);
            std::cout << "リフト準備開始" << std::endl;
            data[23] = speed_lift_up;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            udp.send(data);
            std::cout << "リフト上昇中..." << std::endl;
            data[23] = 0;
            data[33] = 0;
            udp.send(data);
            std::cout << "リフト上昇完了." << std::endl;
    }

    static void lift_down_action(UDP &udp) {
            state_folk_up = true;
            data[33] = 1;
            udp.send(data);
            std::cout << "リフト準備開始" << std::endl;
            data[23] = speed_lift_down;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            udp.send(data);
            std::cout << "リフト下降中..." << std::endl;
            data[23] = 0;
            data[33] = 0;
            udp.send(data);
            std::cout << "リフト下降完了." << std::endl;
    }


    // テスト用！！実機で実行するな！！！！
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
};

bool Action::state_folk_up = false;
bool Action::state_folk_middle = false;
bool Action::state_folk_down = false;

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener(const std::string &ip, int port)
        : Node("nhk25_natsu"), udp_(ip, port) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy0", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "NATSUROBO2025 initialized with IP: %s, Port: %d", ip.c_str(),
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

        //bool LEFT = msg->axes[6] == 1.0;
        //bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        //bool R1 = msg->buttons[5];

        // float L2 = (-1 * msg->axes[2] + 1) / 2;
        // float R2 = (-1 * msg->axes[5] + 1) / 2;

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

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

        if (CIRCLE) {
            Action::collect_ball_action(udp_);
        }

        if (CROSS && Action::state_folk_down) {
            Action::ready_collect_box_action(udp_);
        }
            
        if (CROSS && Action::state_folk_up) {
            Action::collect_box_action(udp_);
        }
        
        if (CROSS && Action::state_folk_middle) {
            Action::end_collect_box_action(udp_);
        }

        if (TRIANGLE){
            Action::shot_action(udp_);
        }

        if (SQUARE){
            Action::shoot_action(udp_);
        }

        if (L1) {
            Action::longshoot_action(udp_);
        }

        if (UP) {
            Action::lift_up_action(udp_);
        }

        if (DOWN) {
            Action::lift_down_action(udp_);
        }

        // if (OPTION) {
        //     Action::tester(udp_);
        // }

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

                    "NATSUROBO2025 Parameter Listener initialized");
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
