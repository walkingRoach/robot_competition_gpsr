//
// Created by ouquanlin on 19-7-30.
//

#ifndef DARKNET_ROS_MSGS_AWAKEN_HPP
#define DARKNET_ROS_MSGS_AWAKEN_HPP

#include <ros/ros.h>

extern "C"{
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "xf/msp_cmn.h"
#include "xf/qivw.h"
#include "xf/msp_errors.h"

#include "xf/formats.h"
#include "xf/linuxrec.h"
#include "xf/speech_recognizer.h"

};
#define SAMPLE_RATE_16K     (16000)
#define E_SR_NOACTIVEDEVICE		1
#define E_SR_NOMEM				2
#define E_SR_INVAL				3
#define E_SR_RECORDFAIL			4
#define E_SR_ALREADY			5

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

void run_ivw(const char *grammar_list ,  const char* session_begin_params);

#endif //DARKNET_ROS_MSGS_AWAKEN_HPP
