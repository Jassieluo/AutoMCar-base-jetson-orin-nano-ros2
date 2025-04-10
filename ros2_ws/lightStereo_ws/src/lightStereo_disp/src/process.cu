#include "lightStereo_disp/process.h"

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

__global__ void uchar2float_kernel(
	uint8_t* src, int src_line_size, int src_width,
	int src_height, float* dst, int dst_width,
	int dst_height, uint8_t const_value_st, int edge) {
	int position = blockDim.x * blockIdx.x + threadIdx.x;
	if (position >= edge) return;

	int dx = position % dst_width;
	int dy = position / dst_width;
	float c0, c1, c2;

	c0 = float(*(src + dy * src_line_size + dx * 3));
	c1 = float(*(src + dy * src_line_size + dx * 3 + 1));
	c2 = float(*(src + dy * src_line_size + dx * 3 + 2));

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

void preprocess_no_resize(
	uint8_t* src, const int& src_width, const int& src_height,
	float* dst, const int& dst_width, const int& dst_height,
	cudaStream_t stream) {

	int jobs = dst_height * dst_width;
	int threads = 256;
	int blocks = ceil(jobs / (float)threads);
	uchar2float_kernel << <blocks, threads, 0, stream >> > (
		src, src_width * 3, src_width,
		src_height, dst, dst_width,
		dst_height, 128, jobs);
}