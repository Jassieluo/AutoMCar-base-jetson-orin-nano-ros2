#pragma once
#include "NvInfer.h"
#include <string>
#include <fstream>
#include <iostream>

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << msg << std::endl;
        }
    }
};

void readEngineFile(const std::string& engine_file, nvinfer1::IRuntime*& runtime, nvinfer1::ICudaEngine*& engine, 
nvinfer1::IExecutionContext*& context);