/*
* 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的
* 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的
* 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h> // for system()
#include <unistd.h>
#include <errno.h>


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include "xf_ros/TTSAction.h"

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

extern "C" {
#include "xf/qtts.h"
#include "xf/msp_cmn.h"
#include "xf/msp_errors.h"
}

std::string ttsResPath;
std::string audioSavePath;
std::string playCommand;
std::string loginParams;
std_msgs::String ttsStr;

using namespace xf_ros;

typedef actionlib::SimpleActionServer<TTSAction> TTSActionServer;
typedef std::shared_ptr<TTSActionServer > TTSActionServerPtr;
TTSActionServerPtr TTSActionServer_;


/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
    char            riff[4];                // = "RIFF"
    int             size_8;                 // = FileSize - 8
    char            wave[4];                // = "WAVE"
    char            fmt[4];                 // = "fmt "
    int             fmt_size;               // = 下一个结构体的大小 : 16

    short int       format_tag;             // = PCM : 1
    short int       channels;               // = 通道数 : 1
    int             samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
    int             avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
    short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
    short int       bits_per_sample;        // = 量化比特数: 8 | 16

    char            data[4];                // = "data";
    int             data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
    { 'R', 'I', 'F', 'F' },
    0,
    {'W', 'A', 'V', 'E'},
    {'f', 'm', 't', ' '},
    16,
    1,
    1,
    16000,
    32000,
    2,
    16,
    {'d', 'a', 't', 'a'},
    0  
};


/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params) {
    int          ret          = -1;
    FILE*        fp           = NULL;
    const char*  sessionID    = NULL;
    unsigned int audio_len    = 0;
    wave_pcm_hdr wav_hdr      = default_wav_hdr;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

    if (NULL == src_text || NULL == des_path) {
        printf("params is error!\n");
        return ret;
    }
    fp = fopen(des_path, "wb");
    if (NULL == fp) {
        printf("open %s error.\n", des_path);
        return ret;
    }
    
    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret) {
        printf("QTTSSessionBegin failed, error code: %d.\n", ret);
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret) {
        printf("QTTSTextPut failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }
    printf("正在合成 ...\n");
    fwrite(&wav_hdr, sizeof(wav_hdr) , 1, fp); //添加wav音频头，使用采样率为16000
    
    while (1) {
        /* 获取合成音频 */
        const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret)
            break;
        if (NULL != data)
        {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len; //计算data_size大小
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
        printf(">");
        usleep(150*1000); //防止频繁占用CPU
    }//合成状态synth_status取值请参阅《讯飞语音云API文档》

    printf("\n");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSAudioGet failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }
    /* 修正wav文件头数据的大小 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
    /* 将修正过的数据写回文件头部,音频文件为wav格式 */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8, sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
    fwrite(&wav_hdr.data_size, sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
    fclose(fp);
    fp = NULL;
    /* 合成完毕 */
    
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionEnd failed, error code: %d.\n", ret);
    }

    return ret;
}


int makeTextToWav(const char* text, const char* filename) {
    int         ret                  = MSP_SUCCESS;
    /* 用户登录 */
    ret = MSPLogin(NULL, NULL, loginParams.c_str());
    if (MSP_SUCCESS != ret) {
        printf("MSPLogin failed, error code: %d.\n", ret);
        MSPLogout(); //退出登录
        return 1;
    }
    /* 文本合成 */
    printf("开始合成 ...\n");
    std::stringstream sessionBeginParams;
    sessionBeginParams << "engine_type = local, voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 100, pitch = 50, rdn = 2, tts_res_path = " << ttsResPath << std::endl;
    std::cout << sessionBeginParams.str() << std::endl;
    ret = text_to_speech(text, filename, sessionBeginParams.str().c_str());
    if (MSP_SUCCESS != ret) {
        printf("text_to_speech failed, error code: %d.\n", ret);
    }
    printf("合成完毕\n");

    MSPLogout(); //退出登录
    return 1;
}

/******************
* play the wav file
******************/
void playAudio(const char* cmd) {
    system(cmd);
}

void TTSCallback(const std_msgs::String::ConstPtr &msg) {
    std::cout << "Got a topic text: " << msg->data.c_str() << std::endl;

    ros::Time time;
    int32_t  f;
    std::stringstream filename, cmd;
    f = ros::Time::now().sec;
    filename << audioSavePath << f << ".wav";
    cmd << playCommand << " " << filename.str();

    makeTextToWav(msg->data.c_str(), filename.str().c_str());
    playAudio(cmd.str().c_str());
}



void toExit() {
    printf("按任意键退出 ...\n");
    getchar();
    MSPLogout(); //退出登录
}


void TTSActionCb() {
    boost::shared_ptr<const TTSGoal> ActionPtr =
            TTSActionServer_->acceptNewGoal();
    ttsStr = ActionPtr->tts_str;

    std::cout << "Got a topic text: " << ttsStr.data.c_str() << std::endl;

    ros::Time time;
    int32_t  f;
    std::stringstream filename;
    f = ros::Time::now().sec;
    filename << audioSavePath << f << ".wav";

    makeTextToWav(ttsStr.data.c_str(), filename.str().c_str());

    TTSResult actionResult;
    printf("[on_result] result returned..\n");

    std_msgs::String msg;

    msg.data = filename.str();
    std::cout << msg.data << std::endl;

    actionResult.audio_path = msg;
    printf("sending...\n");
    TTSActionServer_->setSucceeded(actionResult, "Send tts path.");
    printf("sending finished...\n");
}

void TTSActionPreemptCB() {
    TTSActionServer_->setPreempted();
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "xf_tts");
    ros::NodeHandle n("~");

    n.param("login_param", loginParams, std::string("appid = 123456"));
    n.param("tts_res_path", ttsResPath,
                      std::string("tts_res_path file unknown"));
    n.param("audio_save_path", audioSavePath,
            std::string("audio_save_path file unknown"));
    n.param("play_command", playCommand,
            std::string("play command unknown"));

    std::string TTSActionName;
    n.param("/xf_ros/actions/tts/name", TTSActionName,
                      std::string("/xf_tts/tts_generate"));
    using namespace std;
    cout << TTSActionName << endl;
    TTSActionServer_.reset(
            new TTSActionServer(n, TTSActionName, false));
    TTSActionServer_->registerGoalCallback(
            boost::bind(TTSActionCb));
    TTSActionServer_->registerPreemptCallback(
            boost::bind(TTSActionPreemptCB));
    TTSActionServer_->start();

    ros::Subscriber sub = n.subscribe("/xf/tts/words", 1, TTSCallback);

    ros::spin();

    return 0;
}
