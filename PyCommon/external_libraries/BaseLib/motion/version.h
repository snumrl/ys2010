#pragma once

static const int MOT_RECENT_VERSION=3;

static const char* MOT_VERSION_STRING[]={"v0", "v1", "v2", "mot_version3"};

//MOT_RECENT_VERSION 2


//MOT_RECENT_VERSION 3 -> 2007.7.24
// posture�� translation joint����� �߰��ϸ鼭 ������.

void savePose(Posture& pose, const char* fn);
void loadPose(Posture& pose, const char* fn);