#include "xf_ros/asr.hpp"
#define SAMPLE_RATE_16K     (16000)

namespace xf_ros {

struct result {
    std::string slot;
    std::string content;
    int confidence;
};

std::ostream&operator << (std::ostream &o, const struct result r) {
    o << "slot: " << r.slot << std::endl;
    o << "content: " << r.content << std::endl;
    o << "confidence: " << r.confidence << std::endl;
}

std::vector<int> split(std::string str, char delimiter) {
  std::vector<int> internal;
  std::stringstream ss(str); // Turn the string into a stream.
  std::string tok;

  while(getline(ss, tok, delimiter)) {
    internal.push_back(std::stoi(tok));
  }

  return internal;
}

static void sleep_ms(size_t ms)
{
    usleep(ms*1000);
}


/* the record call back */
static void iat_cb(char *data, unsigned long len, void *user_para)
{
    int errcode;
    const char *session_id = (const char *)user_para;

    if(len == 0 || data == NULL)
        return;
    if(!g_is_awaken_succeed){
        errcode = QIVWAudioWrite(session_id, (const void *)data, len, record_state);
    }
    if (MSP_SUCCESS != errcode)
    {
        printf("QIVWAudioWrite failed! error code:%d\n",errcode);
        int ret = stop_record(recorder);
        if (ret != 0) {
            printf("Stop failed! \n");
            //return -E_SR_RECORDFAIL;
        }
        wait_for_rec_stop(recorder, (unsigned int)-1);
        QIVWAudioWrite(session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST);
        record_state = MSP_AUDIO_SAMPLE_LAST;
        g_is_awaken_succeed = false;
    }
    if(record_state == MSP_AUDIO_SAMPLE_FIRST){
        record_state = MSP_AUDIO_SAMPLE_CONTINUE;
    }
}


int cb_ivw_msg_proc( const char *sessionID, int msg, int param1, int param2, const void *info, void *userData )
{
    if (MSP_IVW_MSG_ERROR == msg) //唤醒出错消息
    {
        printf("\n\nMSP_IVW_MSG_ERROR errCode = %d\n\n", param1);
        g_is_awaken_succeed = false;
        record_state = MSP_AUDIO_SAMPLE_LAST;
    }else if (MSP_IVW_MSG_WAKEUP == msg) //唤醒成功消息
    {
        printf("\n\nMSP_IVW_MSG_WAKEUP result = %s\n\n", (char*)info);
        g_is_awaken_succeed = true;
        record_state = MSP_AUDIO_SAMPLE_LAST;
    }
    int ret = stop_record(recorder);
    if (ret != 0) {
        printf("Stop failed! \n");
    }else{
        printf("stop success\n");
    }
    //wait_for_rec_stop(recorder, (unsigned int)-1);
    //QIVWAudioWrite(sessionID, NULL, 0, MSP_AUDIO_SAMPLE_LAST);
    return 0;
}


void run_ivw(const char *grammar_list ,  const char* session_begin_params)
{
    const char *session_id = NULL;
    int err_code = MSP_SUCCESS;
    char sse_hints[128];

    WAVEFORMATEX wavfmt = DEFAULT_FORMAT;
    wavfmt.nSamplesPerSec = SAMPLE_RATE_16K;
    wavfmt.nAvgBytesPerSec = wavfmt.nBlockAlign * wavfmt.nSamplesPerSec;


//start QIVW
    session_id=QIVWSessionBegin(grammar_list, session_begin_params, &err_code);
    if (err_code != MSP_SUCCESS)
    {
        printf("QIVWSessionBegin failed! error code:%d\n",err_code);
        goto exit;
    }


    err_code = QIVWRegisterNotify(session_id, cb_ivw_msg_proc,NULL);
    if (err_code != MSP_SUCCESS)
    {
        snprintf(sse_hints, sizeof(sse_hints), "QIVWRegisterNotify errorCode=%d", err_code);
        printf("QIVWRegisterNotify failed! error code:%d\n",err_code);
        goto exit;
    }
//start record
    err_code = create_recorder(&recorder, iat_cb, (void*)session_id);
    if (recorder == NULL || err_code != 0) {
        printf("create recorder failed: %d\n", err_code);
        err_code = -E_SR_RECORDFAIL;
        goto exit;
    }

    err_code = open_recorder(recorder, get_default_input_dev(), &wavfmt);
    if (err_code != 0) {
        printf("recorder open failed: %d\n", err_code);
        err_code = -E_SR_RECORDFAIL;
        goto exit;
    }

    err_code = start_record(recorder);
    if (err_code != 0) {
        printf("start record failed: %d\n", err_code);
        err_code = -E_SR_RECORDFAIL;
        goto exit;
    }
    record_state = MSP_AUDIO_SAMPLE_FIRST;


    while(record_state != MSP_AUDIO_SAMPLE_LAST)
    {
        sleep_ms(200); //模拟人说话时间间隙，10帧的音频时长为200ms
        printf("waiting for awaken%d\n", record_state);
    }
    snprintf(sse_hints, sizeof(sse_hints), "success");

    exit:
    if (recorder) {
        if(!is_record_stopped(recorder))
            stop_record(recorder);
        close_recorder(recorder);
        destroy_recorder(recorder);
        recorder = NULL;
    }
    if (NULL != session_id)
    {
        QIVWSessionEnd(session_id, sse_hints);
    }
}


// parse result demo in C++
// but maybe parse it in python is more simple
void parse_result(const char *result) {
    using namespace std;
    string rawtext;
    int totalConfidence;
    vector<struct result> resultList;

    TiXmlDocument doc;
    TiXmlElement *pRoot, *pResult, *pProcess;

    if (result) {
        doc.Parse(result, 0, TIXML_ENCODING_UTF8);
        pRoot = doc.FirstChildElement("nlp");
        if (pRoot) {
            pResult = pRoot->FirstChildElement("result");

            string confidence_ = pResult->FirstChildElement("confidence")->GetText();
            auto confidenceList = split(confidence_, '|');

            int i;
            TiXmlElement *e;
            for (i = 0, e = pResult->FirstChildElement("obj_detect")->FirstChildElement();
                    e != NULL;
                    i += 1, e = e->NextSiblingElement()) {
                struct result r = {
                        e->Value(),
                        e->GetText(),
                        confidenceList[i]
                };
                resultList.push_back(r);
            }

            rawtext = pRoot->FirstChildElement("rawtext")->GetText();
            totalConfidence = atoi(pRoot->FirstChildElement("confidence")->GetText());
        }
        cout << rawtext << endl;
        cout << totalConfidence << endl;

        for (auto r : resultList)
            cout << r << endl;
    }
}

bool get_res_length(const char *result) {
    TiXmlDocument doc;
    TiXmlElement *pRoot, *pResult, *pProcess;

    if (result) {
        doc.Parse(result, 0, TIXML_ENCODING_UTF8);
        pRoot = doc.FirstChildElement("nlp");
        if (pRoot) {
            pResult = pRoot->FirstChildElement("result");

            std::string confidence_ = pResult->FirstChildElement("confidence")->GetText();
            auto confidenceList = split(confidence_, '|');

            if (confidenceList.size() > 5){
                std::cout << "听到了语句" << std::endl;
                return true;
            } else{
                std::cout << "没有听到语句, 语句长度:"<< confidenceList.size() << std::endl;
                return false;
            }
        }
    }
}
// send result
void on_result(const char *result, char is_last, void *ptr) {
    ASR *asr = (ASR*)ptr;

    printf("[on_result] result returned..\n");

    std_msgs::String msg;

    std::stringstream ss;
    ss << result;
    msg.data = ss.str();
    std::cout << msg << std::endl;
    if (asrCloud_ && sendOnce)
    {
        HomeRecognizeResult actionResult;
        actionResult.msg = msg;
        asr->HomeRecognizeActionServer_->setSucceeded(actionResult, "Send bounding boxes.");
        asrCloud_ = false;
        sendOnce = false;
        enable_running_ = false;
        printf("sending...\n");
        printf("sending finished...\n");
    } else if(!asrCloud_ && sendOnce){
        //asr_pub.publish(msg);
        HomeRecognizeResult actionResult;
        actionResult.msg = msg;
        asr->HomeRecognizeActionServer_->setSucceeded(actionResult, "Send bounding boxes.");
        enable_running_ = false;
        using_awake = false;
        printf("sending...\n");
        printf("sending finished...\n");
    }
}
// used for inform
void on_speech_begin() {
    printf("Start Listening...\n");
}

void on_speech_end(int reason) {
    // seem don't have on_sppech_end callback..
    if (reason == END_REASON_VAD_DETECT)
        printf("Speaking end\n\n");
    else
        printf("Recognizer error %d\n\n", reason);
}

void ASR::asr_mic(const char *session_begin_params) {
    int errcode;
    int i = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
            on_result,
            this,
            on_speech_begin,
            on_speech_end
    };

    printf("sr_init...\n");
    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("start listen failed %d\n", errcode);
    }
    /* demo 10 minutes recording */
    //while(i++ < 60 * asrContinueMinutes_)
    while(i++ < continueTime) {
        std::cout << "sleep for a while" << std::endl;
        sleep(1);
    }
    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("stop listening failed %d\n", errcode);
    }

    sr_uninit(&iat);
    printf("sr_uninit end...\n");
}

int build_grm_callback(int ecode, const char *info, void *udata) {
    UserData *grm_data = (UserData *)udata;

    if (NULL != grm_data) {
        grm_data->build_fini = 1;
        grm_data->errcode = ecode;
    }

    if (MSP_SUCCESS == ecode && NULL != info) {
        printf("构建语法成功！ 语法ID:%s\n", info);
        if (NULL != grm_data)
            snprintf(grm_data->grammar_id, MAX_GRAMMARID_LEN - 1, info);
    }
    else
        ROS_ERROR("build grammar failed %d", ecode);

    return 0;
}

int ASR::build_grammar() {
    FILE *grm_file                           = NULL;
    char *grm_content                        = NULL;
    unsigned int grm_cnt_len                 = 0;
    char grm_build_params[MAX_PARAMS_LEN]    = {NULL};
    int ret                                  = 0;

    grm_file = fopen(grmFilePath_.c_str(), "rb");
    if(NULL == grm_file) {
        printf("打开\"%s\"文件失败！[%s]\n", grmFilePath_.c_str(), strerror(errno));
        return -1;
    }

    fseek(grm_file, 0, SEEK_END);
    grm_cnt_len = ftell(grm_file);
    fseek(grm_file, 0, SEEK_SET);

    grm_content = (char *)malloc(grm_cnt_len + 1);
    if (NULL == grm_content)
    {
        printf("内存分配失败!\n");
        fclose(grm_file);
        grm_file = NULL;
        return -1;
    }
    fread((void*)grm_content, 1, grm_cnt_len, grm_file);
    grm_content[grm_cnt_len] = '\0';
    fclose(grm_file);
    grm_file = NULL;

    snprintf(grm_build_params, MAX_PARAMS_LEN - 1,
             "engine_type = local, \
              asr_res_path = %s, sample_rate = %d, \
              grm_build_path = %s, ",
             asrResPath_.c_str(),
             SAMPLE_RATE_16K,
             grmBuildPath_.c_str()
    );
    ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_callback, &asr_data_);

    free(grm_content);
    grm_content = NULL;

    return ret;
}

int ASR::run_awake_with_asr(){
    const char *ssb_param = "ivw_threshold=0:1450,sst=wakeup,ivw_res_path =fo|/home/ubuntu/robot_ros/src/xf_ros/res/ivw/wakeupresource.jet";
    ROS_INFO("start awake");
    // 执行两次
    while (run_awaking){
        run_ivw(NULL, ssb_param);
        ROS_INFO("awake end");
        if(g_is_awaken_succeed){
//            ASR *asr = (ASR*)ptr;
            std_msgs::String ss;
            ss.data = "awake";
            // send awaken succeed message
            HomeRecognizeResult actionResult;
            actionResult.msg = ss;
            this->HomeRecognizeActionServer_->setSucceeded(actionResult, "Send bounding boxes.");
            run_awaking = false;
            using_awake = false;

        }
    }
}

int ASR::run_asr() {
    if(asrCloud_){
        ROS_INFO("start network");
        const char* asr_params =
                "sub = iat, domain = iat, language = en_us, "
                "accent = mandarin, sample_rate = 16000, "
                "result_type = plain, result_encoding = utf8";
        sendOnce = true;
        asr_mic(asr_params);
    } else {
        ROS_INFO("start local");
        char asr_params[MAX_PARAMS_LEN] = {NULL};
        const char *rec_rslt = NULL;
        const char *session_id = NULL;
        const char *asr_audiof = NULL;
        FILE *f_pcm = NULL;
        char *pcm_data = NULL;
        long pcm_count = 0;
        long pcm_size = 0;
        int last_audio = 0;

        int aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
        int ep_status = MSP_EP_LOOKING_FOR_SPEECH;
        int rec_status = MSP_REC_STATUS_INCOMPLETE;
        int rss_status = MSP_REC_STATUS_INCOMPLETE;
        int errcode = -1;
        int aud_src = 0;
        //离线语法识别参数设置
        snprintf(asr_params, MAX_PARAMS_LEN - 1,
                 "engine_type = local, language = en_us, \
              asr_res_path = %s, sample_rate = %d, \
              grm_build_path = %s, local_grammar = %s, \
              result_type = xml, result_encoding = UTF-8, ",
                 asrResPath_.c_str(),
                 SAMPLE_RATE_16K,
                 grmBuildPath_.c_str(),
                 asr_data_.grammar_id
        );
        sendOnce = true;
        asr_mic(asr_params);
    }

    // asr_file(audioFilePath_.c_str(), asr_params);
    return 0;
}

ASR::ASR(ros::NodeHandle nh)
    : nodeHandle_(nh)
{
    ROS_INFO("[XF ASR] Node started.");
    wakeSub = nh.subscribe<std_msgs::String>("/wakeUp", 1, boost::bind(&ASR::wakeUp, this, _1));
//    awakeSub = nh.subscribe<std_msgs::String>("/awake", 1, boost::bind(&ASR::awake, this, _1));
//    ros::Subscriber wake = nh.subscribe<std_msgs::String>("/wakeUp", 60, boost::bind(&ASR::wakeUp, this, _1));
    ROS_INFO("sub start");
    initParameters();
    initXF();
    initActionlib();
    main_thread = std::thread(&ASR::start_asr, this);
}

ASR::~ASR() {
    ROS_INFO("[XF ASR] Node ending...");
    MSPLogout();
}

void ASR::initParameters() {
    nodeHandle_.param("publishers/asr/topic", asrPublisherTopicName_,
                      std::string("/xf_ros/asr_msg"));
    nodeHandle_.param("publishers/asr/queue_size", asrPublisherQueueSize_, 1);
    nodeHandle_.param("publishers/asr/latch", asrPublisherLatch_, false);

    nodeHandle_.param("asr_res_path", asrResPath_,
                      std::string("asr_res_path file unknown"));
    nodeHandle_.param("grm_build_path", grmBuildPath_,
                      std::string("grm_build_path file unknown"));
    nodeHandle_.param("grm_file_path", grmFilePath_,
                      std::string("grm_file_path file unknown"));
    nodeHandle_.param("login_param", loginParams_, std::string("appid = 123456"));
    nodeHandle_.param("audio_file_path", audioFilePath_, std::string("audio file unknown"));
    nodeHandle_.param("asr_continue_minutes", asrContinueMinutes_, 1);

    demoDone_ = false;

    memset(&asr_data_, 0, sizeof(UserData));
}

void ASR::initXF() {
    //第一个参数为用户名，第二个参数为密码，传NULL即可，第三个参数是登录参数
    checkRet(MSPLogin(NULL, NULL, loginParams_.c_str()), "login failed");

    //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
    checkRet(build_grammar(), "build grammar failed");

    while (1 != asr_data_.build_fini)
        usleep(300 * 1000);
    checkRet(asr_data_.errcode);
}

inline void ASR::checkRet(const int ret, const char *errorMsg) {
    if (MSP_SUCCESS != ret) {
        if (errorMsg) {
            ROS_ERROR("[XF ASR] %s error code=%d", errorMsg, ret);
        } else {
            ROS_ERROR("[XF ASR] error code=%d", ret);
        }
        delete this;
    }
}

void ASR::awake(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("start awake service");
    using_awake = true;
//    sendOnce = true;
}

void ASR::wakeUp(const std_msgs::String::ConstPtr& msg) {
//    int child_pid = fork();
//    switch( child_pid ) {
//        case -1:
//            perror( "[fork-exec-test] fork failed" );
//            exit(-1);
//            break;
//        case 0:
////        system("play /home/lennon/Desktop/wav/on.wav tempo 1.2");
//            ROS_DEBUG("KK");
//            exit(0);
//            break;
//        default:
////            ROS_INFO("GET MESSAGE");
//            enable_running_ = true;
//            asrCloud_ = true;
//            sendOnce = true;
//            break;
//    }
    ROS_INFO("GET MESSAGE");
    enable_running_ = true;
    asrCloud_ = true;
//    sendOnce = true;
}

void ASR::initActionlib() {
    std::string HomeRecognizeActionName;
    nodeHandle_.param("/xf_ros/actions/home_asr/name", HomeRecognizeActionName,
            std::string("/xf_asr/home_recognize"));
    HomeRecognizeActionServer_.reset(
            new HomeRecognizeActionServer(nodeHandle_, HomeRecognizeActionName, false));
    HomeRecognizeActionServer_->registerGoalCallback(
            boost::bind(&ASR::HomeRecognizeActionGoalCB, this));
    HomeRecognizeActionServer_->registerPreemptCallback(
            boost::bind(&ASR::HomeRecognizeActionPreemptCB, this));
    HomeRecognizeActionServer_->start();
}

void ASR::HomeRecognizeActionGoalCB() {
    ROS_DEBUG("[ASR] asr start action.");

    boost::shared_ptr<const HomeRecognizeGoal> ActionPtr =
            HomeRecognizeActionServer_->acceptNewGoal();
    model = ActionPtr->model.data;
    continueTime = ActionPtr->continue_time;
    int child_pid = fork();
    switch( child_pid ) {
    case -1:
        perror( "[fork-exec-test] fork failed" );
        exit(-1);
        break;
    case 0:
//        system("play /home/lennon/Desktop/wav/on.wav tempo 1.2");
        ROS_DEBUG("KK");
        exit(0);
        break;
    default:
        std::cout << model << std::endl;
        if (model == "local"){
            sendOnce = true;
            enable_running_ = true;
        } else if(model == "awake"){
            std::cout << "start awake" << std::endl;
            using_awake = true;
            run_awaking = true;
            sendOnce = true;
        }else if (model == "cloud"){
            std::cout << "start cloud" <<std::endl;
            enable_running_ = true;
            asrCloud_ = true;
//            sendOnce = true;
        }
        break;
    }
}

void ASR::HomeRecognizeActionPreemptCB() {
    ROS_DEBUG("[ASR] Preempt home asr action.");
    HomeRecognizeActionServer_->setPreempted();
}

void ASR::start_asr() {
    using namespace std;

    int child_pid = fork();
    switch( child_pid ) {
    case -1:
        perror( "[fork-exec-test] fork failed" );
        exit(-1);
        break;
    case 0:
//        system("play /home/lennon/Desktop/wav/on.wav tempo 1.2");
        exit(0);
        break;
    }

    thread asr_thread;
    // enable_running_ = true;
//    continueTime = 6;
    while (!demoDone_) {
        sleep(0.5);
        if (enable_running_) {
    // run_asr();
            asr_thread = thread(&ASR::run_asr, this);
            asr_thread.join();
//            enable_running_ = false;
            if(!asrCloud_) enable_running_ = false;
        }
        if (using_awake){
            asr_thread = thread(&ASR::run_awake_with_asr, this);
            asr_thread.join();
        }
    }
}

}
