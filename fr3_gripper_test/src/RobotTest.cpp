
#include <iostream>
#include "robot.h"
#include "robot_error.h"
#include <string>
#include <chrono>
#include <thread>

using namespace std;
class RobotTest{
    
private: 
    struct RobotCmd{
        string hold = "HOLD";
        string cup = "CUP";
        string par_1 = "1";
        string par_2 = "2";
        string par_3 = "3";
        string ice = "ICE";
        string unhold = "UNHOLD"; 
    }RobotCmd;
    FRRobot fr;

public:
    RobotTest();
    RobotTest(const char* ip){
        errno_t ret;
        ret = fr.RPC(ip);
        cout << "Connection Robot Result : " << ret << endl;
    }
    ~RobotTest(){
        fr.CloseRPC();
    }
    void GetVersion(){
        char version[100];
        this->fr.GetSDKVersion(version);
        cout << "Robot Version : " << version << endl;
    }
    void GetInverse(JointPos *j, DescPose* d){
        int type = 0;
        int config = -1;
        fr.GetInverseKin(type,d,config,j);
    }
    errno_t base_gripper(){
        errno_t ret =0;
        int index =1;
        int pos = 10;
        int vel = 100;
        int force = 100;
        int max_time = 30000;
        int type_ = 0;
        double rotNum = 0;
        int rotVel = 0;
        int rotTorque = 0;
        uint8_t block = 1;
        ret = fr.MoveGripper(index, pos, vel, force, max_time, block, type_, rotNum, rotVel, rotTorque);

        cout << "BaseMotion Gripper Result : " << ret << endl;
        return ret;
    }
    errno_t open_gripper(){
        errno_t ret =0;
        int index =1;
        int pos = 100;
        int vel = 100;
        int force = 100;
        int max_time = 30000;
        int type_ = 0;
        double rotNum = 0;
        int rotVel = 0;
        int rotTorque = 0;
        uint8_t block = 1;
        ret = fr.MoveGripper(index, pos, vel, force, max_time, block, type_, rotNum, rotVel, rotTorque);

        cout << "Open Gripper Result : " << ret << endl;
        return ret;
    }
    errno_t close_gripper(){
        errno_t ret =0;
        int index =1;
        int pos = 0;
        int vel = 100;
        int force = 100;
        int max_time = 30000;
        int type_ = 0;
        double rotNum = 0;
        int rotVel = 0;
        int rotTorque = 0;
        uint8_t block = 1;
        ret = fr.MoveGripper(index, pos, vel, force, max_time, block, type_, rotNum, rotVel, rotTorque);

        cout << "Close Gripper Result : " << ret << endl;
        return ret;
    }
    /* @brief  관절 공간 운동
        * @param  [in] joint_pos  목표 관절 위치, 단위 deg
        * @param  [in] desc_pos   목표 데카르트 자세
        * @param  [in] tool  도구 좌표 번호, 범위 [1~15]
        * @param  [in] user  작업 좌표 번호, 범위 [1~15]
        * @param  [in] vel  속도 백분율, 범위 [0~100]
        * @param  [in] acc  가속도 백분율, 범위 [0~100], 현재 미지원
        * @param  [in] ovl  속도 스케일링 팩터, 범위 [0~100]
        * @param  [in] epos  확장 축 위치, 단위 mm
        * @param  [in] blendT [-1.0]- 목표 위치 도달(블로킹), [0~500.0]- 부드러운 이동 시간(논블로킹), 단위 ms
        * @param  [in] offset_flag  0- 오프셋 없음, 1- 기본 좌표계/작업 좌표계에서 오프셋, 2- 도구 좌표계에서 오프셋
        * @param  [in] offset_pos  자세 오프셋 값
    */
    void send_moveJ(JointPos* j){
       
        float ovl = 100.0;
        ExaxisPos exis(0.0,0.0,0.0,0.0);
        DescPose desc_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        // JointPos joint_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        float blendR=-1.0;
        int tool =1;
        int user= 0;
        int vel =100;
        int acc =85;
        uint8_t offset_flag = 0;
        DescPose offset_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        // if (cmd == RobotCmd.hold){
        //     if (par1 == RobotCmd.cup){
        //         if(par2 == RobotCmd.par_1){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //         else if(par2 == RobotCmd.par_2){
        //             joint_pos.jPos[0] = 0.1;
        //         }
        //         else if(par2 == RobotCmd.par_3){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //     }
        //     else if (par1 == RobotCmd.ice){
        //         if(par2 == RobotCmd.par_1){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //         else if(par2 == RobotCmd.par_2){
        //             joint_pos.jPos[0] = 0.1;
        //         }
        //         else if(par2 == RobotCmd.par_3){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //     }
        // }
        // else if (cmd == RobotCmd.unhold){
        //     if (par1 == RobotCmd.cup){
        //         if(par2 == RobotCmd.par_1){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //         else if(par2 == RobotCmd.par_2){
        //             joint_pos.jPos[0] = 0.1;
        //         }
        //         else if(par2 == RobotCmd.par_3){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //     }
        //     else if (par1 == RobotCmd.ice){
        //         if(par2 == RobotCmd.par_1){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //         else if(par2 == RobotCmd.par_2){
        //             joint_pos.jPos[0] = 0.1;
        //         }
        //         else if(par2 == RobotCmd.par_3){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //     }
        // }
        errno_t error_check;
        error_check = this->fr.MoveJ(j, &desc_pos, tool, user,  vel, acc, ovl, &exis, blendR, offset_flag, &offset_pos);
        cout << "MOVE_J Check Error : " << error_check << endl; 
    }
    /**
     * @brief  데카르트 공간 직선 운동
     * @param  [in] joint_pos  목표 관절 위치, 단위: deg
     * @param  [in] desc_pos   목표 데카르트 자세
     * @param  [in] tool  도구 좌표 번호, 범위 [1~15]
     * @param  [in] user  작업물 좌표 번호, 범위 [1~15]
     * @param  [in] vel  속도 백분율, 범위 [0~100]
     * @param  [in] acc  가속도 백분율, 범위 [0~100], 현재 미지원
     * @param  [in] ovl  속도 축소 계수, 범위 [0~100]
     * @param  [in] blendR [-1.0]-도착 후 정지(블로킹), [0~1000.0]-부드러운 반경(논블로킹), 단위: mm
     * @param  [in] epos  확장 축 위치, 단위: mm
     * @param  [in] search  0-용접 와이어 탐색 안함, 1-용접 와이어 탐색
     * @param  [in] offset_flag  0-오프셋 없음, 1-기준 좌표계/작업물 좌표계에서 오프셋, 2-도구 좌표계에서 오프셋
     * @param  [in] offset_pos  자세 오프셋량
     * @param  [in] overSpeedStrategy  과속 처리 전략, 1-표준; 2-과속 시 오류 발생 후 정지; 3-자동 감속, 기본값 0
     * @param  [in] speedPercent  허용 감속 임계값 백분율 [0-100], 기본값 10%
    */
    void send_moveL(JointPos *j, DescPose* d){
        float ovl = 100.0;
        ExaxisPos exis(0.0,0.0,0.0,0.0);
        // DescPose desc_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        // JointPos joint_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        float blendR=20.0;
        int tool =1;
        int user= 0;
        int vel =100;
        int acc =85;
        uint8_t offset_flag = 0;
        DescPose offset_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        int overSpeedStrategy = 1;
        int speedPercent= 10;
        uint8_t search = 0;
        // if (cmd == RobotCmd.hold){
        //     if (par1 == RobotCmd.cup){
        //         if(par2 == RobotCmd.par_1){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //         else if(par2 == RobotCmd.par_2){
        //             joint_pos.jPos[0] = 0.1;
        //         }
        //         else if(par2 == RobotCmd.par_3){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //     }
        //     else if (par1 == RobotCmd.ice){
        //         if(par2 == RobotCmd.par_1){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //         else if(par2 == RobotCmd.par_2){
        //             joint_pos.jPos[0] = 0.1;
        //         }
        //         else if(par2 == RobotCmd.par_3){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //     }
        // }
        // else if (cmd == RobotCmd.unhold){
        //     if (par1 == RobotCmd.cup){
        //         if(par2 == RobotCmd.par_1){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //         else if(par2 == RobotCmd.par_2){
        //             joint_pos.jPos[0] = 0.1;
        //         }
        //         else if(par2 == RobotCmd.par_3){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //     }
        //     else if (par1 == RobotCmd.ice){
        //         if(par2 == RobotCmd.par_1){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //         else if(par2 == RobotCmd.par_2){
        //             joint_pos.jPos[0] = 0.1;
        //         }
        //         else if(par2 == RobotCmd.par_3){
        //             joint_pos.jPos[0] = 0.0;
        //         }
        //     }
        // }
        errno_t error_check;
        // JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos, int overSpeedStrategy, int speedPercent)
        error_check = this->fr.MoveL(j, d, tool, user,  vel, acc, ovl, blendR, &exis,search, offset_flag, &offset_pos, overSpeedStrategy, speedPercent);
        cout << "MOVE_L Check : " << error_check << endl;    
    }
};

int main(){
    RobotTest* R = new RobotTest("192.168.58.2");
    // JointPos Front_J(2.235,-86.579,-78.189,-195.23,-87.765, -0.0);
    // JointPos Back_J(5.803, -60.054, -95.343, -204.6, -84.197, 0.0);
    JointPos Back_J;
    JointPos Front_J;
    DescPose Front_L(483.5, -93.3, 380.5, 90.0, 0.0, 90.0);
    DescPose Back_L(353.1, -93.314, 380.5, 90.0, 0.0,90.0);
    // R->send_moveJ(&Cup_APP);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // R->send_moveJ(&Cup1_APP_J);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    int cnt = 5;
    int type = 0;
    int config = -1;
    while(cnt>0){
        R->GetInverse(&Back_J, &Back_L);
        R->send_moveL(&Back_J, &Back_L);
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        R->open_gripper();
        R->GetInverse(&Front_J, &Front_L);
        R->send_moveL(&Front_J, &Front_L);
        
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        R->close_gripper();
        cnt--;
    }
    R->send_moveL(&Back_J, &Back_L);
    // R->send_moveJ(&Cup_APP);
    //RobotTest R("192.168.0.11");
    //R.fr.ActGripper(1,1);
    return 0;
}