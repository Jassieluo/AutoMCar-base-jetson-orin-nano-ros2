#include "yolo_detect/process.h"


__global__ void warpaffine_kernel(
	uint8_t* src, int src_line_size, int src_width,
	int src_height, float* dst, int dst_width,
	int dst_height, uint8_t const_value_st,
	AffineMatrix d2s, int edge) {
	int position = blockDim.x * blockIdx.x + threadIdx.x;
	if (position >= edge) return;

	float m_x1 = d2s.value[0];
	float m_y1 = d2s.value[1];
	float m_x2 = d2s.value[3];
	float m_y2 = d2s.value[4];

	int dx = position % dst_width;
	int dy = position / dst_width;
	float src_x = m_x1 * dx + m_y1 * dy;
	float src_y = m_x2 * dx + m_y2 * dy;
	float c0, c1, c2;

	if (src_x < 0 || src_x + 1 >= src_width || src_y < 0 || src_y + 1 >= src_height) {
		c0 = const_value_st;
		c1 = const_value_st;
		c2 = const_value_st;
	}
	else {
		int x_low = floorf(src_x);
		int y_low = floorf(src_y);
		int x_high = x_low + 1;
		int y_high = y_low + 1;
		float w1 = (y_high - src_y) * (x_high - src_x);
		float w2 = (y_high - src_y) * (src_x - x_low);
		float w3 = (src_y - y_low) * (x_high - src_x);
		float w4 = (src_y - y_low) * (src_x - x_low);
		uint8_t* v1 = src + y_low * src_line_size + x_low * 3;
		uint8_t* v2 = src + y_low * src_line_size + x_high * 3;
		uint8_t* v3 = src + y_high * src_line_size + x_low * 3;
		uint8_t* v4 = src + y_high * src_line_size + x_high * 3;
		c0 = w1 * v1[0] + w2 * v2[0] + w3 * v3[0] + w4 * v4[0];
		c1 = w1 * v1[1] + w2 * v2[1] + w3 * v3[1] + w4 * v4[1];
		c2 = w1 * v1[2] + w2 * v2[2] + w3 * v3[2] + w4 * v4[2];
	}

	// bgr -> rgb
	float temp = c2;
	c2 = c0;
	c0 = temp;

	// normalization
	c0 /= 255.0f;
	c1 /= 255.0f;
	c2 /= 255.0f;
	// rgbrgbrgb -> rrrgggbbb
	int area = dst_height * dst_width;
	float* pdst_c0 = dst + dy * dst_width + dx;
	float* pdst_c1 = pdst_c0 + area;
	float* pdst_c2 = pdst_c1 + area;
	*pdst_c0 = c0;
	*pdst_c1 = c1;
	*pdst_c2 = c2;
}

void preprocess(
	uint8_t* src, const int& src_width, const int& src_height,
	float* dst, const int& dst_width, const int& dst_height,
	cudaStream_t stream, float& scale) {

	AffineMatrix s2d, d2s;
	scale = std::min(dst_height / (float)src_height, dst_width / (float)src_width);
	s2d.value[0] = scale;
	s2d.value[1] = 0;
	s2d.value[2] = 0;
	s2d.value[3] = 0;
	s2d.value[4] = scale;
	s2d.value[5] = 0;
	cv::Mat m2x3_s2d(2, 3, CV_32F, s2d.value);
	cv::Mat m2x3_d2s(2, 3, CV_32F, d2s.value);
	cv::invertAffineTransform(m2x3_s2d, m2x3_d2s);

	memcpy(d2s.value, m2x3_d2s.ptr<float>(0), sizeof(d2s.value));

	int jobs = dst_height * dst_width;
	int threads = 256;
	int blocks = ceil(jobs / (float)threads);
	warpaffine_kernel << <blocks, threads, 0, stream >> > (
		src, src_width * 3, src_width,
		src_height, dst, dst_width,
		dst_height, 128, d2s, jobs);
}

__global__ void select_bbox(int kClass, int num_ancors, int keep_ancors, float conf, float* src, float* dst, int* pointer, int xyxy)
{
	int idx = blockDim.x * blockIdx.x + threadIdx.x;
	int classid = 0;
	float maxConf = 0;
	if (idx >= num_ancors)
	{
		return;
	}
	for (int i = 4; i < kClass; ++i)
	{
		if (src[i * num_ancors + idx] >= maxConf)
		{
			maxConf = src[i * num_ancors + idx];
			classid = i - 4;
		}
	}
	if (maxConf >= conf)
	{
		if (*pointer >= keep_ancors * 6) return;
		int pointeridx = atomicAdd(pointer, 6);
		if (!xyxy)
		{
			for (int i = 0; i < 4; ++i)
			{
				dst[pointeridx + i] = src[idx + i * num_ancors];
			}
		}
		else
		{
			dst[pointeridx] = src[idx] - src[idx + 2 * num_ancors] / 2;
			dst[pointeridx + 2] = src[idx] + src[idx + 2 * num_ancors] / 2;
			dst[pointeridx + 1] = src[idx + 1 * num_ancors] - src[idx + 3 * num_ancors] / 2;
			dst[pointeridx + 3] = src[idx + 1 * num_ancors] + src[idx + 3 * num_ancors] / 2;
		}
		dst[pointeridx + 4] = src[idx + (4 + classid) * num_ancors];
		dst[pointeridx + 5] = classid;
	}
}

void postprocess(int kClass, int num_ancors, int keep_ancors, float conf, float* src, float* dst_host, int xyxy, cudaStream_t stream)
{
	int point = -6;
	int* pointer;
	pointer = &point;
	int* pointer_device;
	float* dst;
	int threads = 512;
	int blocks = ceil(num_ancors / threads);
	cudaMalloc((void**)&pointer_device, sizeof(int));
	cudaMemcpy(pointer_device, pointer, sizeof(int), cudaMemcpyHostToDevice);
	cudaMalloc((void**)&dst, sizeof(float) * keep_ancors * 6);
	select_bbox << <blocks, threads, 0, stream>> > (kClass, num_ancors, keep_ancors, conf, src, dst, pointer_device, xyxy);
	cudaMemcpyAsync(dst_host + 1, dst, sizeof(float) * keep_ancors * 6, cudaMemcpyDeviceToHost, stream);
	cudaMemcpyAsync(pointer, pointer_device, sizeof(int), cudaMemcpyDeviceToHost, stream);
	*dst_host = ceil(((*pointer) + 6) / 6) + 1;
}

static float iou(float lbox[4], float rbox[4]) {
	float interBox[] = {
	  (std::max)(lbox[0] - lbox[2] / 2.f , rbox[0] - rbox[2] / 2.f), //left
	  (std::min)(lbox[0] + lbox[2] / 2.f , rbox[0] + rbox[2] / 2.f), //right
	  (std::max)(lbox[1] - lbox[3] / 2.f , rbox[1] - rbox[3] / 2.f), //top
	  (std::min)(lbox[1] + lbox[3] / 2.f , rbox[1] + rbox[3] / 2.f), //bottom
	};

	if (interBox[2] > interBox[3] || interBox[0] > interBox[1])
		return 0.0f;

	float interBoxS = (interBox[1] - interBox[0]) * (interBox[3] - interBox[2]);
	return interBoxS / (lbox[2] * lbox[3] + rbox[2] * rbox[3] - interBoxS);
}

static bool cmp(const Detection& a, const Detection& b) {
	return a.conf > b.conf;
}


void NMS(std::vector<Detection>& res, float* output, const float& conf_thresh, const float& nms_thresh) {
	int det_size = sizeof(Detection) / sizeof(float);
	std::map<float, std::vector<Detection>> m;
	for (int i = 0; i < output[0]; i++) {
		//for (int j = 0; j < 6; ++j) std::cout << output[1 + det_size * i + j] << " ";
		if (output[1 + det_size * i + 4] <= conf_thresh) continue;
		Detection det;
		memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
		/*std::cout << det.conf << " ";*/
		if (m.count(det.class_id) == 0) m.emplace(det.class_id, std::vector<Detection>());
		m[det.class_id].push_back(det);
	}
	for (auto it = m.begin(); it != m.end(); it++) {
		auto& dets = it->second;
		std::sort(dets.begin(), dets.end(), cmp);
		for (size_t m = 0; m < dets.size(); ++m) {
			auto& item = dets[m];
			res.push_back(item);
			for (size_t n = m + 1; n < dets.size(); ++n) {
				if (iou(item.bbox, dets[n].bbox) > nms_thresh) {
					dets.erase(dets.begin() + n);
					--n;
				}
			}
		}
	}
}

void NMS_DetMsg(detect_msgs::msg::DetectMsgs& detect_msg_res, float* output, const float& conf_thresh, const float& nms_thresh) {
	int det_size = sizeof(Detection) / sizeof(float);
	std::map<float, std::vector<Detection>> m;
	for (int i = 0; i < output[0]; i++) {
		//for (int j = 0; j < 6; ++j) std::cout << output[1 + det_size * i + j] << " ";
		if (output[1 + det_size * i + 4] <= conf_thresh) continue;
		Detection det;
		memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
		/*std::cout << det.conf << " ";*/
		if (m.count(det.class_id) == 0) m.emplace(det.class_id, std::vector<Detection>());
		m[det.class_id].push_back(det);
	}
	for (auto it = m.begin(); it != m.end(); it++) {
		auto& dets = it->second;
		std::sort(dets.begin(), dets.end(), cmp);
		for (size_t m = 0; m < dets.size(); ++m) {
			auto& item = dets[m];
			detect_msgs::msg::DetectBaseMsg item_detmsg;
			item_detmsg.bbox[0] = item.bbox[0];
			item_detmsg.bbox[1] = item.bbox[1];
			item_detmsg.bbox[2] = item.bbox[2];
			item_detmsg.bbox[3] = item.bbox[3];
			item_detmsg.class_id = item.class_id;
			item_detmsg.conf = item.conf;
			detect_msg_res.detect_msg.push_back(item_detmsg);
			for (size_t n = m + 1; n < dets.size(); ++n) {
				if (iou(item.bbox, dets[n].bbox) > nms_thresh) {
					dets.erase(dets.begin() + n);
					--n;
				}
			}
		}
	}
}

//cv::Rect getRect(cv::Mat& img, float bbox[4], float& scale) {
//	float l, r, t, b;
//	l = bbox[0] / scale;
//	t = bbox[1] / scale;
//	r = bbox[2] / scale;
//	b = bbox[3] / scale;
//	return cv::Rect(int(l), int(t), int(r - l), int(b - t));
//}
//
//void drawBbox(cv::Mat& img, std::vector<Detection>& res, float& scale, std::map<int, std::string>& Labels) {
//	for (size_t j = 0; j < res.size(); j++) {
//		cv::Rect r = getRect(img, res[j].bbox, scale);
//		std::string name = Labels[(int)res[j].class_id];
//		cv::rectangle(img, r, cv::Scalar(0xFF, 0xFF, 0), 2);
//		cv::putText(img, name, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0), 2);
//	}
//}

//void drawBbox(cv::Mat& img, std::vector<Detection>& res, float& scale, std::map<int, std::string>& Labels) {
//	for (size_t j = 0; j < res.size(); j++) {
//		float l = res[j].bbox[0] / scale;
//		float t = res[j].bbox[1] / scale;
//		float r = res[j].bbox[2] / scale;
//		float b = res[j].bbox[3] / scale;
//		cv::Rect rect = cv::Rect(int(l), int(t), int(r - l), int(b - t));
//		std::string name = Labels[(int)res[j].class_id];
//		cv::rectangle(img, rect, cv::Scalar(0xFF, 0xFF, 0), 2);
//		cv::putText(img, name, cv::Point(rect.x, rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0), 2);
//	}
//}

std::string to_string_with_precision(float num, int precision) {
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(precision) << num;
	return oss.str();
}

std::vector<cv::Scalar> palette = {
	  cv::Scalar(220, 20, 60), cv::Scalar(119, 11, 32), cv::Scalar(0, 0, 142), cv::Scalar(0, 0, 230), cv::Scalar(106, 0, 228),
	  cv::Scalar(0, 60, 100), cv::Scalar(0, 80, 100), cv::Scalar(0, 0, 70), cv::Scalar(0, 0, 192), cv::Scalar(250, 170, 30),
	  cv::Scalar(100, 170, 30), cv::Scalar(220, 220, 0), cv::Scalar(175, 116, 175), cv::Scalar(250, 0, 30),
	  cv::Scalar(165, 42, 42), cv::Scalar(255, 77, 255), cv::Scalar(0, 226, 252), cv::Scalar(182, 182, 255),
	  cv::Scalar(0, 82, 0), cv::Scalar(120, 166, 157), cv::Scalar(110, 76, 0), cv::Scalar(174, 57, 255),
	  cv::Scalar(199, 100, 0), cv::Scalar(72, 0, 118), cv::Scalar(255, 179, 240), cv::Scalar(0, 125, 92),
	  cv::Scalar(209, 0, 151), cv::Scalar(188, 208, 182), cv::Scalar(0, 220, 176), cv::Scalar(255, 99, 164),
	  cv::Scalar(92, 0, 73), cv::Scalar(133, 129, 255), cv::Scalar(78, 180, 255), cv::Scalar(0, 228, 0),
	  cv::Scalar(174, 255, 243), cv::Scalar(45, 89, 255), cv::Scalar(134, 134, 103), cv::Scalar(145, 148, 174),
	  cv::Scalar(255, 208, 186), cv::Scalar(197, 226, 255), cv::Scalar(171, 134, 1), cv::Scalar(109, 63, 54),
	  cv::Scalar(207, 138, 255), cv::Scalar(151, 0, 95), cv::Scalar(9, 80, 61), cv::Scalar(84, 105, 51),
	  cv::Scalar(74, 65, 105), cv::Scalar(166, 196, 102), cv::Scalar(208, 195, 210), cv::Scalar(255, 109, 65),
	  cv::Scalar(0, 143, 149), cv::Scalar(179, 0, 194), cv::Scalar(209, 99, 106), cv::Scalar(5, 121, 0),
	  cv::Scalar(227, 255, 205), cv::Scalar(147, 186, 208), cv::Scalar(153, 69, 1), cv::Scalar(3, 95, 161),
	  cv::Scalar(163, 255, 0), cv::Scalar(119, 0, 170), cv::Scalar(0, 182, 199), cv::Scalar(0, 165, 120),
	  cv::Scalar(183, 130, 88), cv::Scalar(95, 32, 0), cv::Scalar(130, 114, 135), cv::Scalar(110, 129, 133),
	  cv::Scalar(166, 74, 118), cv::Scalar(219, 142, 185), cv::Scalar(79, 210, 114), cv::Scalar(178, 90, 62),
	  cv::Scalar(65, 70, 15), cv::Scalar(127, 167, 115), cv::Scalar(59, 105, 106), cv::Scalar(142, 108, 45),
	  cv::Scalar(196, 172, 0), cv::Scalar(95, 54, 80), cv::Scalar(128, 76, 255), cv::Scalar(201, 57, 1),
	  cv::Scalar(246, 0, 122), cv::Scalar(191, 162, 208)
};

void drawBbox(cv::Mat& img, std::vector<Detection>& res, float& scale, std::map<int, std::string>& Labels) {
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 0.7;
	int thickness = 1;
	int baseline = 0;
	for (size_t j = 0; j < res.size(); j++) {
		float l = res[j].bbox[0] / scale;
		float t = res[j].bbox[1] / scale;
		float r = res[j].bbox[2] / scale;
		float b = res[j].bbox[3] / scale;
		cv::Rect rect = cv::Rect(int(l), int(t), int(r - l), int(b - t));
		std::string name = Labels[(int)res[j].class_id] + ": ";
		std::string text = name + to_string_with_precision((float)res[j].conf, 2);
		cv::rectangle(img, rect, palette[(int)res[j].class_id], 2);
		cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
		cv::Rect backgroundRect(cv::Point2i(rect.x, rect.y - textSize.height), textSize);
		cv::rectangle(img, backgroundRect, palette[(int)res[j].class_id], cv::FILLED);
		cv::putText(img, text, cv::Point(rect.x, rect.y - 1), fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
	}
}

void drawBbox_DetMsg(cv::Mat& img, detect_msgs::msg::DetectMsgs& detect_msg_res, float& scale, std::map<int, std::string>& Labels) {
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 0.7;
	int thickness = 1;
	int baseline = 0;
	for (size_t j = 0; j < detect_msg_res.detect_msg.size(); j++) {
		float l = detect_msg_res.detect_msg[j].bbox[0] / scale;
		float t = detect_msg_res.detect_msg[j].bbox[1] / scale;
		float r = detect_msg_res.detect_msg[j].bbox[2] / scale;
		float b = detect_msg_res.detect_msg[j].bbox[3] / scale;
		cv::Rect rect = cv::Rect(int(l), int(t), int(r - l), int(b - t));
		std::string name = Labels[(int)detect_msg_res.detect_msg[j].class_id] + ": ";
		std::string text = name + to_string_with_precision((float)detect_msg_res.detect_msg[j].conf, 2);
		cv::rectangle(img, rect, palette[(int)detect_msg_res.detect_msg[j].class_id], 2);
		cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
		cv::Rect backgroundRect(cv::Point2i(rect.x, rect.y - textSize.height), textSize);
		cv::rectangle(img, backgroundRect, palette[(int)detect_msg_res.detect_msg[j].class_id], cv::FILLED);
		cv::putText(img, text, cv::Point(rect.x, rect.y - 1), fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
	}
}