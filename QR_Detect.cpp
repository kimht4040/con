#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>



using namespace std;
using namespace cv;
using namespace zbar;


// 전역 변수 및 공유 자원
Mat shared_frame;          // 스레드 간에 공유할 카메라 프레임
mutex frame_mtx;           // 프레임 접근 보호를 위한 뮤텍스
atomic<bool> is_running(true); // 프로그램 종료 플래그
atomic<bool> arm_is_run(false); // 팔 제어 플래그
atomic<bool> is_belt_run(true); //벨트 상황 플래그
int s;
struct sockaddr_can addr;
struct ifreq ifr;




// 카메라 입력 및 qr 계산 스레드
void vision_thread_func() {
    
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "camera error" << endl;
        is_running = false;
        return;
    }
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    Mat local_frame, gray;

    while (is_running) {
        cap >> local_frame;
        if (local_frame.empty()) continue;
        
        bool is_qr_seen_this_frame = false;
       // zbar qr 인식 
        cvtColor(local_frame, gray, COLOR_BGR2GRAY);
        int width = gray.cols;
        int height = gray.rows;
        zbar::Image zbar_img(width, height, "Y800", gray.data, width * height);
        
        scanner.scan(zbar_img);

        // 2/3 지점 가로선 미리 계산
        int line_y = static_cast<int>(height * (2.0 / 3.0));
        line(local_frame, Point(0, line_y), Point(width, line_y), Scalar(0, 0, 255), 2);
        struct can_frame frame;
        // QR 인식 결과 처리
        for (Image::SymbolIterator symbol = zbar_img.symbol_begin(); symbol != zbar_img.symbol_end(); ++symbol) {
            string data = symbol->get_data();

            if (symbol->get_location_size() == 4) {
                vector<Point> vp;
                for (int i = 0; i < 4; i++) {
                    vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
                }
                
                polylines(local_frame, vp, true, Scalar(0, 255, 0), 2);
                putText(local_frame, data, vp[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);

     
                int centerX = (vp[0].x + vp[1].x + vp[2].x + vp[3].x) / 4;
                int centerY = (vp[0].y + vp[1].y + vp[2].y + vp[3].y) / 4;
                circle(local_frame, Point(centerX, centerY), 4, Scalar(0, 0, 255), -1);

      
                if (abs(centerY - line_y) < 20) {
                    is_qr_seen_this_frame = true;
                    if (data == "stop") {
                        
                        if(is_belt_run == true){
                            cout << "belt_stop" << endl;
                            frame.can_id = 0x124;  // 벨트쪽 id
                            frame.can_dlc = 1;    
                            frame.data[0] = 0xAA;  // 벨트 정지 명령 데이터 
                            if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
                                perror("Write error");
                                is_running = false;
                                return;  
                            }
                            is_belt_run = false;
                        }


                        
                        //로봇팔 구동
                        if(arm_is_run == false){
                            cout << "arm_run" << endl;
                            frame.can_id = 0x123; // 로봇팔 아이디
                            frame.can_dlc = 1;   
                            frame.data[0] = 0xAB; // 로봇팔 구동 명령 데이터
                            if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
                                perror("Write error");
                                is_running = false; // 플래그를 false로 만들어 안전하게 종료
                                return;          
                            }
                            arm_is_run = true; // 이후 false로 변경은 arm에서 동작완료 can 신호 보내면 false로 변경함
                            // 그전까진 재동작 신호 전송x
                        }




                    } else if (data == "trash") {
                        cout << "trash" << endl;
                        // 분류기 구동신호 벨트는 그대로 동작하고 분류기만 작동
                        frame.can_id = 0x124;  // 벨트쪽 id
                        frame.can_dlc = 1;     
                        frame.data[0] = 0xAC;  // 분류기 구동 데이터 
                        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
                            perror("Write error");
                            is_running = false;
                            return;  
                        }
                    }
                    else
                        cout << "pass" << endl;
                }


            }
        }
        // 2. 메인 스레드가 화면에 그릴 수 있도록 공유 변수에 복사 (뮤텍스 락 필요)
        {
            lock_guard<mutex> lock(frame_mtx);
            local_frame.copyTo(shared_frame);
        }
    }

    cap.release();
}
//  메인에서 화면 출력이랑 can 수신 담당
int main() {

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW))<0){
        perror("error for socket");
        return -1;

    }
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 소켓 바인딩
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in bind");
        return -1;
    }

    // 로봇팔 수신 필터 부분
    struct can_filter rfilter[2];
    rfilter[0].can_id   = 0x125;
    rfilter[0].can_mask = CAN_SFF_MASK;
    // belt_fillter
    rfilter[1].can_id   = 0x126;
    rfilter[1].can_mask = CAN_SFF_MASK; 

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));



    //소켓 논블로킹 모드 변경
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    // 비전 스레드 생성 및 실행
    thread vision_worker(vision_thread_func);

    Mat display_frame;
    struct can_frame frame;
    while (is_running) {
        // 공유 프레임 안전하게 가져오기
        {
            lock_guard<mutex> lock(frame_mtx);
            if (!shared_frame.empty()) {
                shared_frame.copyTo(display_frame);
            }
        }

        // 프레임이 비어있지 않으면 화면에 출력
        if (!display_frame.empty()) {
            imshow("Multi-threaded Conveyor QR Scanner", display_frame);
        }

        if (waitKey(10) == 'q') {
            is_running = false;
            break;
        }
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes > 0 && nbytes == sizeof(struct can_frame)) {
            std::cout << "특정 ID(0x" << std::hex << frame.can_id 
                    << ")로부터 신호가 수신되었습니다!" << std::endl;

            if (frame.can_id == 0x125) {
                arm_is_run = false;   // 로봇팔 동작 완료 신호
            } else if (frame.can_id == 0x126) {
                is_belt_run = true; // 벨트 재동작 신호

            }
        } else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("Read error");
            break;
        }
    }

    if (vision_worker.joinable()) {
        vision_worker.join();
    }

    destroyAllWindows();
    cout << "exit program" << endl;

    close(s);
    return 0;
}


void salfkjkasdfl() {//asdfasdf

}
