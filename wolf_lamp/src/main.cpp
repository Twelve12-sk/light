#include<opencv2/opencv.hpp>
#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<iostream>
#include <time.h>
#include "authlib.hpp"
#include "serialib.hpp"

using namespace cv;
using namespace std;
using namespace al;
using namespace sl;

sl::Serialib serial("/dev/ttyUSB0", 115200);

// 声明函数
void move_lamp(int lamp_m_h, int lamp_m_w, int bg_m_h, int bg_m_w, Mat& mask, Mat& lamp);
void move(int& lamp_m_h, int& lamp_m_w, int& lamp_m_h_unit, int& lamp_m_w_unit, int& bg_m_h, int& bg_m_w, int& bg_m_h_unit, int& bg_m_w_unit);
Mat make_mask(Mat lamp);
int randNum(int min_val, int max_val);
int test_send(string s);
int test_read();
void no();
void run();
int check(vector<char> str, int start);

char read_text[] ={'X', 'Y', 'R', 'G', 'B'};
char check_text[] = {'9', '0', '1', '2', '2'};

// ---------- 自行修改 -----------------
//int bg_h = 1440, bg_w = 900;        //bg图片大小
//int lamp_h = 950, lamp_w = 450;     //lamp图片大小（调整）
int bg_h = 720, bg_w = 450;        //bg图片大小
int lamp_h = 475, lamp_w = 225;     //lamp图片大小（调整）

//  载入图片 "路径"  ------ 自行修改 --------------
String BG = "/home/pi/wsl_new_folder/RM21队内赛信号灯/dist/BG.png";
String GREEN_filename = "/home/pi/wsl_new_folder/RM21队内赛信号灯/dist/GREEN_cut.png";
String GREEN_yellow_on_filename = "/home/pi/wsl_new_folder/RM21队内赛信号灯/dist/GREEN(yellow-on)_cut.png";
String RED_filename = "/home/pi/wsl_new_folder/RM21队内赛信号灯/dist/RED_cut.png";
String RED_yellow_on_filename = "/home/pi/wsl_new_folder/RM21队内赛信号灯/dist/RED(yellow-on)_cut.png";

// BG图片全局
Mat BG_image;

int main()
{
    cv::namedWindow("output", WINDOW_NORMAL);
    cv::setWindowProperty("output", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
    run();
}

void run()
{
    serial.flush();
    // -------------------- 先把图片全部加载完 ----------------------------
    BG_image = imread(BG);
    resize(BG_image, BG_image, Size(BG_image.cols / 3, BG_image.rows / 3));

    Mat lamp_G = imread(GREEN_filename);
    resize(lamp_G, lamp_G, Size(lamp_G.cols / 2.5, lamp_G.rows / 2.5));
    Mat mask_G = make_mask(lamp_G);

    Mat lamp_G_y = imread(GREEN_yellow_on_filename);
    resize(lamp_G_y, lamp_G_y, Size(lamp_G_y.cols / 2.5, lamp_G_y.rows / 2.5));
    Mat mask_G_y = make_mask(lamp_G_y);

    Mat lamp_R = imread(RED_filename);
    resize(lamp_R, lamp_R, Size(lamp_R.cols / 2.5, lamp_R.rows / 2.5));
    Mat mask_R = make_mask(lamp_R);

    Mat lamp_R_y = imread(RED_yellow_on_filename);
    resize(lamp_R_y, lamp_R_y, Size(lamp_R_y.cols / 2.5, lamp_R_y.rows / 2.5));
    Mat mask_R_y = make_mask(lamp_R_y);
    // -------------------- 防止反复 imread 卡顿 ---------------------------


    // -----------------------后续修改建议使用结构体--------------------------
    int lamp_m_h = 100, lamp_m_w = 100;         //lamp_ROI移动起始点
    int lamp_m_h_unit = 2, lamp_m_w_unit = 3;   //lamp移动单位

    int bg_m_h = 65, bg_m_w = 65;                 //BG_ROI移动起始点
    int bg_m_h_unit = -4, bg_m_w_unit = -3;       //BG移动单位
    // -----------------------后续修改建议使用结构体--------------------------

    //待机画面
    move_lamp(lamp_m_h, lamp_m_w, bg_m_h, bg_m_w, mask_G, lamp_G);
    test_read();

    for(;;)   //capture.read(frame)
    {
        int latency_time;
        int flag = 0;
        //发送切换红绿灯信号
        test_send("G");
        latency_time = randNum(46, 115);              //获得范围内的随机数
        for (int i = 0 ; i< latency_time; i++)        //60帧下, 循环60次大概需要 1.31秒         =>         换算下来即为 45.8015次 循环的时间 花费1秒
        {
            move_lamp(lamp_m_h, lamp_m_w, bg_m_h, bg_m_w, mask_G, lamp_G);
            move(lamp_m_h, lamp_m_w, lamp_m_h_unit, lamp_m_w_unit, bg_m_h, bg_m_w, bg_m_h_unit, bg_m_w_unit);

            //达到某个时间后，读取传入的判断信息
            if (i > 45 && flag == 0)
            {
                test_read();
                flag++;
            }
        }
        flag = 0;       //重置flag

        for (int j = 0; j < 2; j++)
        {
            int l = 0;
            while (l < 5)
            {
                move_lamp(lamp_m_h, lamp_m_w, bg_m_h, bg_m_w, mask_G_y, lamp_G_y);
                move(lamp_m_h, lamp_m_w, lamp_m_h_unit, lamp_m_w_unit, bg_m_h, bg_m_w, bg_m_h_unit, bg_m_w_unit);
                l++;
            }
            while (l < 10)
            {
                move_lamp(lamp_m_h, lamp_m_w, bg_m_h, bg_m_w, mask_G, lamp_G);
                move(lamp_m_h, lamp_m_w, lamp_m_h_unit, lamp_m_w_unit, bg_m_h, bg_m_w, bg_m_h_unit, bg_m_w_unit);
                l++;
            }
        }

        //发送切换红绿灯信号
        test_send("R");
        latency_time = randNum(46, 115);            //获得范围内的随机数
        for (int i = 0; i < latency_time; i++)
        {
            move_lamp(lamp_m_h, lamp_m_w, bg_m_h, bg_m_w, mask_R, lamp_R);
            move(lamp_m_h, lamp_m_w, lamp_m_h_unit, lamp_m_w_unit, bg_m_h, bg_m_w, bg_m_h_unit, bg_m_w_unit);

            //达到某个时间后，读取传入的判断信息
            if (i > 35 && flag == 0)
            {
                test_read();
                flag++;
            }
        }
        for (int j = 0; j < 2; j++)
        {
            int l = 0;
            while(l < 5)
            {
                move_lamp(lamp_m_h, lamp_m_w, bg_m_h, bg_m_w, mask_R_y, lamp_R_y);
                move(lamp_m_h, lamp_m_w, lamp_m_h_unit, lamp_m_w_unit, bg_m_h, bg_m_w, bg_m_h_unit, bg_m_w_unit);
                l++;
            }
            while(l<10)
            {
                move_lamp(lamp_m_h, lamp_m_w, bg_m_h, bg_m_w, mask_R, lamp_R);
                move(lamp_m_h, lamp_m_w, lamp_m_h_unit, lamp_m_w_unit, bg_m_h, bg_m_w, bg_m_h_unit, bg_m_w_unit);
                l++;
            }
        }
    }
}

// imread 处理太慢了，会导致帧数低
// 移动lamp显示
void move_lamp(int lamp_m_h, int lamp_m_w, int bg_m_h, int bg_m_w, Mat& mask, Mat& lamp)
{
    int64 time0 = getTickCount();
    //(不能用)浅拷贝, 当图像之间进行赋值时，图像数据并未发生复制，两个对象指向同一块内存, 改变图像2会影响图像 1
    //Mat image_2 = image;

    //深拷贝, 当图像之间进行赋值时，图像数据发生复制，两个对象指向不同的内存, 改变图像2不会影响图像 1
    //img.copyTo(image);
    Mat BG_ROI = BG_image(Rect(bg_m_h, bg_m_w, 2880/3 - 80 - 47, 1800/3 - 80 ));
    static Mat BG_image_2(cv::Size(BG_ROI.size()), CV_8UC3);//= BG_ROI.clone();
    BG_ROI.copyTo(BG_image_2);

    static Mat imROI;
    imROI = BG_image_2(Rect(lamp_m_h, lamp_m_w, lamp.cols, lamp.rows));
    //image.copyTo(imageROI，mask), 作用是把mask和image重叠以后把mask中像素值为0（black）的点对应的image中的点变为透明，而保留其他点。
    lamp.copyTo(imROI, mask);
    imshow("output", BG_image_2);

    waitKey(1);
    //测试帧数相关
    for(int flag=0 ; flag==0 ; )
    {
        if(getTickFrequency() / (getTickCount() - time0) <= 60)
        {
            flag = 1;
        }
    }
    cout << "当前帧率：" << getTickFrequency() / (getTickCount() - time0) << endl;

}

// -------------------- 限制移动范围，自行修改 -------------------------------
// 反弹路径
void move(int &lamp_m_h, int &lamp_m_w, int &lamp_m_h_unit, int &lamp_m_w_unit, int &bg_m_h, int &bg_m_w, int &bg_m_h_unit, int &bg_m_w_unit)
{
    //lamp_ROI点 移动区域
    lamp_m_h += lamp_m_h_unit;
    lamp_m_w += lamp_m_w_unit;

    //高度达到上下限则反弹
    if (lamp_m_h >= bg_h - lamp_h - 120  ||  lamp_m_h <= 30 )
    {
        lamp_m_h_unit = -lamp_m_h_unit;
        lamp_m_h += 2 * lamp_m_h_unit;
    }
    //宽度达到上下限反弹
    if (lamp_m_w >= bg_w - lamp_w - 50  ||  lamp_m_w <= 30 )
    {
        lamp_m_w_unit = -lamp_m_w_unit;
        lamp_m_w += 2 * lamp_m_w_unit;
    }


    //BG_ROI 移动区域
    bg_m_h += bg_m_h_unit;
    bg_m_w += bg_m_w_unit;

    if (bg_m_h >= 75  ||  bg_m_h <= 2)
    {
        bg_m_h_unit = -bg_m_h_unit;
        bg_m_h += 2 * bg_m_h_unit;
    }
    if (bg_m_w >= 75  ||  bg_m_w <= 2)
    {
        bg_m_w_unit = -bg_m_w_unit;
        bg_m_w += 2 * bg_m_w_unit;
    }
}

// 做掩膜
Mat make_mask(Mat lamp)
{
    Mat grayPng;
    cvtColor(lamp, grayPng, COLOR_BGR2GRAY);
    threshold(grayPng, grayPng, 10, 255, THRESH_BINARY);
    //掩模反色
    //Mat mask = 255 - grayPng;
    Mat mask = grayPng;
    //imshow("mask", mask);

    return mask;
}

//@brief:产生[min_val,max_val]范围内的随机数
//@param:min_val:最小值
//@param:max_val:最大值
//@return:生成的随机数
int randNum(int min_val, int max_val)
{
    //判断前检查 min_val 和 max_val的值大小对比
    if (min_val > max_val)
    {
        cout << "不符合要求，min_val必须小等于max_val的值。" << endl;
        return 0;
    }
    //
    int num_val = rand() % (max_val - min_val + 1) + min_val;
    return num_val;
}

//发送数据
int test_send(string s)
{
    vector<char> str(s.begin(), s.end());
    vector<char> checksum;

//    checksum << al::CRC8_MAXIM(str);
    //给 校验位 赋值
    if(s == "R")
    {
        checksum.push_back(check_text[2]);
    }
    else if(s == "G")
    {
        checksum.push_back(check_text[3]);
    }
    else if(s == "Y")
    {
        checksum.push_back(check_text[1]);
    }
    else if(s == "X")
    {
        checksum.push_back(check_text[0]);
    }
    //发多点，gkd
    int times = 10;
    while(times --)
    {
        serial << ("S" | str | checksum | "E");
    }

    return 0;
}


//逻辑漏洞->（检测到一次SxxE后后面的SxxE就不管了，会出现问题）
//尝试一直读取，检测到帧头帧尾
int test_read()
{
    vector<char> str;
    cout << "read:";

    while( str.empty() )
    {
        waitKey(5);
        serial >> str;
        cout << "正在读取" << endl;
    }

    int start , end;    int i=-1;
    int read_flag = 1;
    while(read_flag != 0 && i<=38) //后面条件防止串口读取完全也没得到想要的<数据>内容
    {
        //逐为读取，查看帧start头end尾，当头尾符合的时候，打印出中间内容
        do
        {
            // i 表示当前读取到的位置
            i++;
            start = i;
            end = start + 3; //+4
        }while( ! (str[start] == 'S' && str[end] == 'E') );

        //输出除去帧头尾后的 <数据+校验> 内容
        for(int i=start+1 ; i<=start+2 ; i++) //+3
        {
            cout << str[i];
        }cout << endl;

        // ----------------
        // 校验区
        if( check(str,start) )
        {
            //确认接受正确,根据传输数据内容，执行操作
            if( str[start + 1] == 'X' )
            {
                test_send("X");
                no();
            }
            else if( str[start + 1] == 'Y' )
            {
                read_flag = 0; //跳出read
            }
            else if( str[start + 1] == 'B' )
            {
                //判断启动
               read_flag = 0; //跳出read
            }
            else //发过来的数据不是想要的，继续读取串口，全部数据都不是想要的->程序崩溃
            {
                i = end;
            }
        }
        else
        {
            //检验判断串口传输出错，则继续读取串口
            i = end;
        }
    }
    // ----------------

    // 车不出错就继续

    // 清除缓存区内容
    waitKey(1);
    serial.flush();

    return 1;
}


void no()
{
    Mat error = imread("/home/pi/wsl_new_folder/RM21队内赛信号灯/dist/error.png");
    imshow("output",error);
    waitKey(1000);
    //回调
    run();
}


//检测数据传输是否出错
int check(vector<char> str, int start)
{
    for(int i=0 ; i<= 4 ; i++)
    {
        if( str[start+1] == read_text[i] )
        {
            if( str[start+2] == check_text[i] )
            {
                //检验通过，返回1
                return 1;
            }
        }
    }
    //检验未通过，返回0
    return 0;
}

