#include "tcp_objectDetection/utils.h"

void readClassFile(const std::string& class_file, std::map<int, std::string>& labels) {
    std::fstream file(class_file, std::ios::in);
    if (!file.is_open()) {
        std::cout << "Load classes file failed: " << class_file << std::endl;
        system("pause");
        exit(0);
    }
    std::cout << "Load classes file success: " << class_file << std::endl;
    std::string str_line;
    int index = 0;
    while (getline(file, str_line)) {
        labels.insert({ index, str_line });
        index++;
    }
    file.close();
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

void drawBbox_DetMsg(cv::Mat& img, detect_msgs::msg::DetectMsgs& detect_msg_res, std::map<int, std::string>& Labels) {
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 0.7;
	int thickness = 1;
	int baseline = 0;
	for (size_t j = 0; j < detect_msg_res.detect_msg.size(); j++) {
		float l = detect_msg_res.detect_msg[j].bbox[0];
		float t = detect_msg_res.detect_msg[j].bbox[1];
		float r = detect_msg_res.detect_msg[j].bbox[2];
		float b = detect_msg_res.detect_msg[j].bbox[3];
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