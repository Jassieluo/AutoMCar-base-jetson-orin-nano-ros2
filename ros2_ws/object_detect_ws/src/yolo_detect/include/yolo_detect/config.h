#pragma once
#define USE_FP16
//#define USE_INT8

const static char* kInputTensorName = "images";
const static char* kOutputTensorName = "output";

const static int kNumClass = 80;
const static int kBatchSize = 1;

const static int kInputH = 256;
const static int kInputW = 512;

const static float kNmsThresh = 0.45f;
const static float kConfThresh = 0.25f;

const static int kMaxInputImageSize = kInputW*kInputH;
const static int kMaxNumOutputBbox = 50;

const static int kNumAnchors = 2688;

struct alignas(float) Detection {
  float bbox[4];
  float conf;
  float class_id;
};
