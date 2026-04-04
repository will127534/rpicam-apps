/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * lut3d_stage.cpp - 3D LUT post-processing stage for film emulation
 *
 * Applies a 3D color lookup table (.cube format) to the camera output.
 * Supports runtime LUT switching via a control file for live preview.
 *
 * The 9x9x9 LUT is expanded to a 256x256x256 precomputed table at load
 * time (~48MB) for O(1) per-pixel lookup with no runtime trilinear math.
 *
 * JSON config:
 * {
 *     "lut3d": {
 *         "lut_directory": "/home/pi/luts/3dlut",
 *         "lut_name": "3DLUT6",
 *         "control_file": "/tmp/rpicam_lut",
 *         "enabled": true
 *     }
 * }
 *
 * Runtime LUT switching: echo "3DLUT10" > /tmp/rpicam_lut
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

using Stream = libcamera::Stream;

class Lut3dStage : public PostProcessingStage
{
public:
	Lut3dStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Configure() override;
	bool Process(CompletedRequestPtr &completed_request) override;

private:
	bool loadCubeFile(const std::string &path);
	void buildFullLut();
	void checkControlFile();

	// Trilinear interpolation on the source 3D LUT
	void trilinearLookup(float fr, float fg, float fb, float &outR, float &outG, float &outB) const;

	Stream *stream_;
	StreamInfo info_;

	std::string lut_directory_;
	std::string lut_name_;
	std::string control_file_;
	bool enabled_;

	// Source LUT: lut_size_^3 entries, each (R,G,B) in [0,1]
	int lut_size_;
	std::vector<std::array<float, 3>> lut_data_;

	// Precomputed full 256^3 lookup: maps (R,G,B) -> (R',G',B') directly
	// Stored as interleaved R,G,B bytes: full_lut_[r][g][b*3+ch]
	// Total: 256*256*768 = ~48MB
	std::vector<uint8_t> full_lut_;
	bool lut_ready_;

	// Precomputed YUV<->RGB tables for fast conversion
	// yuv_to_rgb_lut_[Y][U][V] -> packed RGB (but 256^3 = 48MB, too big for two)
	// Instead, precompute the combined YUV->LUT->YUV transform:
	// For Y channel: we need per-pixel. For UV: we average a 2x2 block.

	// Control file polling
	unsigned int frame_count_;
	static constexpr unsigned int CHECK_INTERVAL = 15;

	std::mutex lut_mutex_;

	// YUV <-> RGB (BT.601 full range) using fixed-point
	static inline void yuvToRgb(int Y, int U, int V, int &R, int &G, int &B)
	{
		R = Y + ((1436 * (V - 128)) >> 10);
		G = Y - ((352 * (U - 128) + 731 * (V - 128)) >> 10);
		B = Y + ((1815 * (U - 128)) >> 10);
		R = std::clamp(R, 0, 255);
		G = std::clamp(G, 0, 255);
		B = std::clamp(B, 0, 255);
	}

	static inline void rgbToYuv(int R, int G, int B, int &Y, int &U, int &V)
	{
		Y = ((306 * R + 601 * G + 117 * B) >> 10);
		U = ((-173 * R - 339 * G + 512 * B) >> 10) + 128;
		V = ((512 * R - 429 * G - 83 * B) >> 10) + 128;
		Y = std::clamp(Y, 0, 255);
		U = std::clamp(U, 0, 255);
		V = std::clamp(V, 0, 255);
	}
};

#define NAME "lut3d"

char const *Lut3dStage::Name() const
{
	return NAME;
}

void Lut3dStage::Read(boost::property_tree::ptree const &params)
{
	lut_directory_ = params.get<std::string>("lut_directory", "/home/pi/luts/3dlut");
	lut_name_ = params.get<std::string>("lut_name", "3DLUT0");
	control_file_ = params.get<std::string>("control_file", "/tmp/rpicam_lut");
	enabled_ = params.get<bool>("enabled", true);
	lut_ready_ = false;
	frame_count_ = 0;
}

void Lut3dStage::Configure()
{
	stream_ = app_->GetMainStream();
	info_ = app_->GetStreamInfo(stream_);

	std::string path = lut_directory_ + "/" + lut_name_ + ".cube";
	if (!loadCubeFile(path))
		fprintf(stderr, "lut3d: failed to load %s\n", path.c_str());

	// Write initial status
	std::ofstream ctl(control_file_);
	if (ctl.is_open())
		ctl << lut_name_ << std::endl;

	fprintf(stderr, "lut3d: configured with %s (%dx%dx%d), format=%s\n",
			lut_name_.c_str(), lut_size_, lut_size_, lut_size_,
			info_.pixel_format.toString().c_str());
}

bool Lut3dStage::loadCubeFile(const std::string &path)
{
	std::ifstream file(path);
	if (!file.is_open())
		return false;

	std::vector<std::array<float, 3>> data;
	int size = 0;
	std::string line;

	while (std::getline(file, line))
	{
		if (line.empty() || line[0] == '#')
			continue;
		if (line.find("LUT_3D_SIZE") != std::string::npos)
		{
			std::istringstream iss(line);
			std::string tag;
			iss >> tag >> size;
			continue;
		}
		if (line.find("TITLE") != std::string::npos ||
			line.find("DOMAIN") != std::string::npos)
			continue;

		std::istringstream iss(line);
		float r, g, b;
		if (iss >> r >> g >> b)
			data.push_back({r, g, b});
	}

	if (size <= 0 || (int)data.size() != size * size * size)
	{
		fprintf(stderr, "lut3d: invalid cube file %s (size=%d, entries=%zu)\n",
				path.c_str(), size, data.size());
		return false;
	}

	{
		std::lock_guard<std::mutex> lock(lut_mutex_);
		lut_size_ = size;
		lut_data_ = std::move(data);
		buildFullLut();
		lut_ready_ = true;
	}
	return true;
}

void Lut3dStage::trilinearLookup(float fr, float fg, float fb,
								  float &outR, float &outG, float &outB) const
{
	int r0 = std::min((int)fr, lut_size_ - 2);
	int g0 = std::min((int)fg, lut_size_ - 2);
	int b0 = std::min((int)fb, lut_size_ - 2);

	float dr = fr - r0;
	float dg = fg - g0;
	float db = fb - b0;

	// .cube: R varies fastest
	auto idx = [this](int ri, int gi, int bi) {
		return ri + gi * lut_size_ + bi * lut_size_ * lut_size_;
	};

	const auto &c000 = lut_data_[idx(r0, g0, b0)];
	const auto &c100 = lut_data_[idx(r0 + 1, g0, b0)];
	const auto &c010 = lut_data_[idx(r0, g0 + 1, b0)];
	const auto &c110 = lut_data_[idx(r0 + 1, g0 + 1, b0)];
	const auto &c001 = lut_data_[idx(r0, g0, b0 + 1)];
	const auto &c101 = lut_data_[idx(r0 + 1, g0, b0 + 1)];
	const auto &c011 = lut_data_[idx(r0, g0 + 1, b0 + 1)];
	const auto &c111 = lut_data_[idx(r0 + 1, g0 + 1, b0 + 1)];

	float w000 = (1 - dr) * (1 - dg) * (1 - db);
	float w100 = dr * (1 - dg) * (1 - db);
	float w010 = (1 - dr) * dg * (1 - db);
	float w110 = dr * dg * (1 - db);
	float w001 = (1 - dr) * (1 - dg) * db;
	float w101 = dr * (1 - dg) * db;
	float w011 = (1 - dr) * dg * db;
	float w111 = dr * dg * db;

	for (int ch = 0; ch < 3; ch++)
	{
		float v = c000[ch] * w000 + c100[ch] * w100 + c010[ch] * w010 + c110[ch] * w110 +
				  c001[ch] * w001 + c101[ch] * w101 + c011[ch] * w011 + c111[ch] * w111;
		if (ch == 0) outR = v;
		else if (ch == 1) outG = v;
		else outB = v;
	}
}

void Lut3dStage::buildFullLut()
{
	// Precompute 256x256x256 -> RGB lookup
	// Layout: full_lut_[r * 256 * 256 * 3 + g * 256 * 3 + b * 3 + ch]
	const size_t total = 256 * 256 * 256 * 3;
	full_lut_.resize(total);

	const float scale = (lut_size_ - 1) / 255.0f;

	for (int r = 0; r < 256; r++)
	{
		for (int g = 0; g < 256; g++)
		{
			for (int b = 0; b < 256; b++)
			{
				float outR, outG, outB;
				trilinearLookup(r * scale, g * scale, b * scale, outR, outG, outB);

				size_t idx = ((size_t)r * 256 * 256 + g * 256 + b) * 3;
				full_lut_[idx + 0] = std::clamp((int)(outR * 255.0f + 0.5f), 0, 255);
				full_lut_[idx + 1] = std::clamp((int)(outG * 255.0f + 0.5f), 0, 255);
				full_lut_[idx + 2] = std::clamp((int)(outB * 255.0f + 0.5f), 0, 255);
			}
		}
	}

	fprintf(stderr, "lut3d: built full 256^3 lookup table (%.1f MB)\n", total / 1e6);
}

void Lut3dStage::checkControlFile()
{
	std::ifstream ctl(control_file_);
	if (!ctl.is_open())
		return;

	std::string cmd;
	std::getline(ctl, cmd);
	ctl.close();

	while (!cmd.empty() && (cmd.back() == '\n' || cmd.back() == '\r' || cmd.back() == ' '))
		cmd.pop_back();

	if (cmd.empty() || cmd == lut_name_)
		return;

	if (cmd == "off")
	{
		enabled_ = false;
		fprintf(stderr, "lut3d: disabled\n");
		return;
	}
	if (cmd == "on")
	{
		enabled_ = true;
		fprintf(stderr, "lut3d: enabled (%s)\n", lut_name_.c_str());
		return;
	}
	if (cmd == "list")
	{
		DIR *dir = opendir(lut_directory_.c_str());
		if (dir)
		{
			fprintf(stderr, "lut3d: available LUTs:\n");
			struct dirent *entry;
			while ((entry = readdir(dir)) != nullptr)
			{
				std::string name(entry->d_name);
				if (name.size() > 5 && name.substr(name.size() - 5) == ".cube")
					fprintf(stderr, "  %s%s\n", name.substr(0, name.size() - 5).c_str(),
							(name.substr(0, name.size() - 5) == lut_name_) ? " [active]" : "");
			}
			closedir(dir);
		}
		std::ofstream out(control_file_);
		if (out.is_open())
			out << lut_name_ << std::endl;
		return;
	}

	std::string path = lut_directory_ + "/" + cmd + ".cube";
	if (loadCubeFile(path))
	{
		lut_name_ = cmd;
		fprintf(stderr, "lut3d: switched to %s\n", cmd.c_str());
	}
	else
	{
		fprintf(stderr, "lut3d: failed to load %s\n", path.c_str());
		std::ofstream out(control_file_);
		if (out.is_open())
			out << lut_name_ << std::endl;
	}
}

bool Lut3dStage::Process(CompletedRequestPtr &completed_request)
{
	if (++frame_count_ >= CHECK_INTERVAL)
	{
		frame_count_ = 0;
		checkControlFile();
	}

	if (!enabled_ || !lut_ready_)
		return false;

	std::lock_guard<std::mutex> lock(lut_mutex_);

	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint8_t *data = buffer.data();
	const uint8_t *lut = full_lut_.data();

	unsigned int width = info_.width;
	unsigned int height = info_.height;
	unsigned int stride = info_.stride;

	bool is_rgb = (info_.pixel_format == libcamera::formats::RGB888 ||
				   info_.pixel_format == libcamera::formats::BGR888);
	bool is_bgr = (info_.pixel_format == libcamera::formats::BGR888);

	if (is_rgb)
	{
		// Direct RGB processing — no YUV conversion needed
		for (unsigned int row = 0; row < height; row++)
		{
			uint8_t *p = data + row * stride;
			for (unsigned int col = 0; col < width; col++, p += 3)
			{
				int r = is_bgr ? p[2] : p[0];
				int g = p[1];
				int b = is_bgr ? p[0] : p[2];

				size_t idx = ((size_t)r * 256 * 256 + g * 256 + b) * 3;
				uint8_t nr = lut[idx + 0];
				uint8_t ng = lut[idx + 1];
				uint8_t nb = lut[idx + 2];

				if (is_bgr)
				{
					p[0] = nb;
					p[1] = ng;
					p[2] = nr;
				}
				else
				{
					p[0] = nr;
					p[1] = ng;
					p[2] = nb;
				}
			}
		}
	}
	else
	{
		// YUV420 processing — convert per 2x2 block
		unsigned int y_plane_size = height * stride;
		unsigned int uv_stride = stride / 2;

		uint8_t *Y_plane = data;
		uint8_t *U_plane = data + y_plane_size;
		uint8_t *V_plane = U_plane + (height / 2) * uv_stride;

		for (unsigned int row = 0; row < height - 1; row += 2)
		{
			uint8_t *Y0 = Y_plane + row * stride;
			uint8_t *Y1 = Y0 + stride;
			uint8_t *U = U_plane + (row / 2) * uv_stride;
			uint8_t *V = V_plane + (row / 2) * uv_stride;

			for (unsigned int col = 0; col < width - 1; col += 2)
			{
				int u = U[col / 2];
				int v = V[col / 2];
				int R, G, B, nY, nU, nV;
				int sumU = 0, sumV = 0;

				// Pixel (0,0)
				yuvToRgb(Y0[col], u, v, R, G, B);
				{
					size_t idx = ((size_t)R * 256 * 256 + G * 256 + B) * 3;
					rgbToYuv(lut[idx], lut[idx + 1], lut[idx + 2], nY, nU, nV);
				}
				Y0[col] = nY;
				sumU += nU;
				sumV += nV;

				// Pixel (0,1)
				yuvToRgb(Y0[col + 1], u, v, R, G, B);
				{
					size_t idx = ((size_t)R * 256 * 256 + G * 256 + B) * 3;
					rgbToYuv(lut[idx], lut[idx + 1], lut[idx + 2], nY, nU, nV);
				}
				Y0[col + 1] = nY;
				sumU += nU;
				sumV += nV;

				// Pixel (1,0)
				yuvToRgb(Y1[col], u, v, R, G, B);
				{
					size_t idx = ((size_t)R * 256 * 256 + G * 256 + B) * 3;
					rgbToYuv(lut[idx], lut[idx + 1], lut[idx + 2], nY, nU, nV);
				}
				Y1[col] = nY;
				sumU += nU;
				sumV += nV;

				// Pixel (1,1)
				yuvToRgb(Y1[col + 1], u, v, R, G, B);
				{
					size_t idx = ((size_t)R * 256 * 256 + G * 256 + B) * 3;
					rgbToYuv(lut[idx], lut[idx + 1], lut[idx + 2], nY, nU, nV);
				}
				Y1[col + 1] = nY;
				sumU += nU;
				sumV += nV;

				U[col / 2] = (sumU + 2) >> 2;
				V[col / 2] = (sumV + 2) >> 2;
			}
		}
	}

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new Lut3dStage(app);
}

static RegisterStage reg(NAME, &Create);
