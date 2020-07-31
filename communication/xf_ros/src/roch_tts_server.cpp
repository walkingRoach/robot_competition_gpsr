#include <stdlib.h> // for system()
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>


#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include <ros/ros.h>
//#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>
#include <roch_tts/RochTTSAction.h>

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int		        size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int		        fmt_size;		        // = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int		        samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		        avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		        data_size;              // = 纯数据长度 : FileSize - 44 
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


class RochTTSAction
{
protected:
	ros::NodeHandle nh_;

	actionlib::SimpleActionServer<roch_tts::RochTTSAction> as_;
	std::string action_name_;
	roch_tts::RochTTSFeedback feedback_;
	roch_tts::RochTTSResult result_;
	ros::Subscriber sub_;

public:

	RochTTSAction(std::string name);
	virtual ~RochTTSAction(void);

	void executeCB(const roch_tts::RochTTSGoalConstPtr &goal);
	void PlayWav(const char* cmd);
	int makeTextToWav(const char* text, const char* filename);
	int text_to_speech(const char* src_text, const char* des_path, const char* params);
private:
	/* data */
};

RochTTSAction::RochTTSAction(std::string name): 
	as_(nh_, name, boost::bind(&RochTTSAction::executeCB, this, _1), false),
	action_name_(name)
	{
		as_.start();
	}

RochTTSAction::~RochTTSAction(void)
{
}


//action_goal回调
void RochTTSAction::executeCB(const roch_tts::RochTTSGoalConstPtr &goal)
{
	ros::Rate r(1);
	bool success = true;

	printf("\n%s: 正在执行中...\n", action_name_.c_str());
	printf("%s: 收到一条消息: %s\n", action_name_.c_str(), goal->roch_tts_goal.c_str());
	
	std::stringstream filename, play_path;
	filename << "/home/vance/sawyer_ws/src/roch_GPSR/roch_tts/src/tts.wav";
    play_path << "play " << filename.str();
	
	if (as_.isPreemptRequested() || !ros::ok())	{
		printf("%s: Preempted(被占用)...\n", action_name_.c_str());
		as_.setPreempted();
		success = false;
		result_.roch_tts_result = false;
	}
	
    makeTextToWav(goal->roch_tts_goal.c_str(), filename.str().c_str()); //语音合成
    PlayWav(play_path.str().c_str()); //语音播放

	if (success) 
		result_.roch_tts_result = true;
	
	as_.setSucceeded(result_);	
}

//音频播放
void RochTTSAction::PlayWav(const char* cmd) 
{
    system(cmd);
	printf("%s: 播放完毕...\n", action_name_.c_str());
}

//音频生成
int RochTTSAction::makeTextToWav(const char* text, const char* filename)
{
	int         ret                  = MSP_SUCCESS; //语音合成成功返回值
	const char* login_params         = "appid = 5972ada2, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	const char* session_begin_params = "voice_name = xiaowanzi, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 100, pitch = 50, rdn = 0";
    /* 用户登录 */
    ret = MSPLogin(NULL, NULL, login_params);
    if (MSP_SUCCESS != ret) {
        printf("%s: MSP登录失败,错误代码: %d.\n", action_name_.c_str(), ret);
        goto exit ;//登录失败，退出登录
    }
    /* 文本合成 */
    printf("%s: 开始合成...\n", action_name_.c_str());
    ret = text_to_speech(text, filename, session_begin_params);
    if (MSP_SUCCESS != ret) {
        printf("%s: 合成失败,错误代码: %d.\n", action_name_.c_str(), ret);
    }
    printf("%s: 合成完毕...\n", action_name_.c_str());

exit:
    MSPLogout(); //退出登录
    return 1;
}

/* 文本合成 */
int RochTTSAction::text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path) {
		printf("%s: params is error!\n", action_name_.c_str());
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp) {
		printf("%s: params is error!\n", action_name_.c_str());
		return ret;
	}
	
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)	{
		printf("%s: QTTSSessionBegin failed, error code: %d.\n", action_name_.c_str(), ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)	{
		printf("%s: QTTSTextPut failed, error code: %d.\n", action_name_.c_str(), ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	
	fwrite(&wav_hdr, sizeof(wav_hdr) , 1, fp); //添加wav音频头，使用采样率为16000
	
	while (1) {
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data) {
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
		printf("%s: QTTSAudioGet failed, error code: %d.\n", action_name_.c_str(), ret);
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
		printf("%s: QTTSSessionEnd failed, error code: %d.\n", action_name_.c_str(), ret);
	}

	return ret;
}


/*void toExit()
{
    printf("按任意键退出 ...\n");
    getchar();
    MSPLogout(); //退出登录
}
*/

/*********************
**  主函数
*********************/
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "roch_tts_server");
	
	RochTTSAction roch_tts("roch_tts_action");

	ros::spin();

	return 0;
}
