#ifndef PROJECT_XF_ASR_HPP
#define PROJECT_XF_ASR_HPP

// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/server/simple_action_server.h>
#include "xf_ros/HomeRecognizeAction.h"

// extra lib
#include <thread>
#include "tinyxml.h"

// xf
extern "C" {
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "xf/qise.h"
#include "xf/qisr.h"
#include "xf/qivw.h"
#include "xf/msp_cmn.h"
#include "xf/msp_errors.h"
#include "xf/speech_recognizer.h"
#include "xf/linuxrec.h"
#include "xf/formats.h"
}

namespace xf_ros {

// define publisher for inconvience communication with xf lib and C++ class
// so make it global in namespace xf_ros and init in asr_node.cpp main function

#define DEFAULT_FORMAT		\
{\
	WAVE_FORMAT_PCM,	\
	1,			\
	16000,			\
	32000,			\
	2,			\
	16,			\
	sizeof(WAVEFORMATEX)	\
}

static int record_state = MSP_AUDIO_SAMPLE_CONTINUE;
struct recorder *recorder;
bool g_is_awaken_succeed = true;

ros::Publisher asr_pub;
ros::Subscriber wakeSub, awakeSub;
bool sendOnce = true;
bool enable_running_ = false;
bool asrCloud_;
bool using_awake = false;
bool run_awaking = false;
bool using_mix = false;

const int FRAME_LEN          = 640;
const int BUFFER_SIZE        = 4096;
const int SAMPLE_RATE_16K    = 16000;
const int SAMPLE_RATE_8K     = 8000;
const int MAX_GRAMMARID_LEN  = 32;
const int MAX_PARAMS_LEN     = 1024;

typedef struct _UserData {
    int     build_fini;     //标识语法构建是否完成
    int     update_fini;    //标识更新词典是否完成
    int     errcode;        //记录语法构建或更新词典回调错误码
    char    grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
}UserData;

class ASR {
public:
    explicit ASR(ros::NodeHandle nh);
    ~ASR();

    int run_asr();

    void start_asr();

    int run_awake_with_asr();

    typedef actionlib::SimpleActionServer<HomeRecognizeAction> HomeRecognizeActionServer;
    typedef std::shared_ptr<HomeRecognizeActionServer> HomeRecognizeActionServerPtr;
    HomeRecognizeActionServerPtr HomeRecognizeActionServer_;
    ros::NodeHandle nodeHandle_;

    bool demoDone_;
    void wakeUp(const std_msgs::String::ConstPtr& msg);
    void awake(const std_msgs::String::ConstPtr& msg);

private:
    void initXF();
    void initParameters();

    inline void checkRet(const int ret, const char* errorMsg = nullptr);

    ros::Publisher asrPublisher_;
    std::string asrPublisherTopicName_;
    int asrPublisherQueueSize_;
    bool asrPublisherLatch_;

    //离线语法识别资源路径
    std::string asrResPath_;
    //构建离线语法识别网络生成数据保存路径
    std::string grmBuildPath_;
    //构建离线识别语法网络所用的语法文件
    std::string grmFilePath_;
    //login所用的param
    std::string loginParams_;
    std::string audioFilePath_;

    // 持续进行asr的时间
    int asrContinueMinutes_;

    //是否使用在线进行识

    // HomeRecognize action related
    void initActionlib();
    void HomeRecognizeActionGoalCB();
    void HomeRecognizeActionPreemptCB();
    std::string model;
    int continueTime;

    // using network pub
    void initSub();


    std::thread main_thread;

    UserData asr_data_;


    // xf lib related
    int build_grammar();
    void asr_mic(const char *session_begin_params);
};
}

#endif //PROJECT_XF_ASR_HPP
