#include "lightStereo_disp/utils.h"

Logger ggLogger;

void readEngineFile(const std::string& engine_file, nvinfer1::IRuntime*& runtime, nvinfer1::ICudaEngine*& engine, nvinfer1::IExecutionContext*& context){
    std::fstream file;

    file.open(engine_file, std::ios::in | std::ios::binary);

    if(!file.is_open()){
        std::cout << "Load engine file failed: " << engine_file << std::endl;
		system("pause");
		exit(0);
    }
    std::cout << "Load engine file success: " << engine_file << std::endl;
	size_t size = 0;
	file.seekg(0, file.end);
	size = file.tellg();
	file.seekg(0, file.beg);
	char* serialized_engine = new char[size];
	file.read(serialized_engine, size);
	file.close();

	runtime = nvinfer1::createInferRuntime(ggLogger);
	engine = runtime->deserializeCudaEngine(serialized_engine, size);
	context = engine->createExecutionContext();
	delete[] serialized_engine;
}