/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * camera helper for imx477 sensor
 */

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include "cam_helper.h"
//#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}


/*
 * VD55G1 has four contexts available for different settings during streaming.
 * The host configures the sequence to switch from one context to the next.
 * The sensor sends X frames from one context, then switches to the Y context.
 * X and Y are the values of the registers CONTEXT_REPEAT_COUNT_CTX[0,1,2,3]
 * and CONTEXT_NEXT_CONTEXT.CTX[0,1,2,3].
 * The first frame output by the sensor is based on context 0 parameters.
 * If CONTEXT_REPEAT_COUNT is set to 0, the sensor stays on that context.
 *
 * Each of the stream contexts appears to support different:
 *	- exposure mode, settings, gain, etc
 *	- cropping
 *	- GPIO behavior
 *	- subsampling & binning / READOUT_CTRL
 *
 */

/*
constexpr uint32_t vtAnalogGainReg = 0x0057; // five bits used
constexpr uint32_t ispDigitalGainCh0HiReg = 0x0058; // integer part
constexpr uint32_t ispDigitalGainCh0LoReg = 0x0059; // fractional part
*/

/*
 * Exposure mode control:
 * 0x0 = automatic
 * 0x1 = freeze AE with current settings
 * 0x2 = manual setting mode
 * 0x4 = bypass
 */
/*
constexpr uint32_t exposureModeReg = 0x0060;
constexpr uint32_t exposureStatusAReg = 0x0061;
constexpr uint32_t exposureMeanEnergyAReg = 0x0062;
constexpr uint32_t ispExposureDigitalGainHiReg = 0x0066; // integer part
constexpr uint32_t ispExposureDigitalGainLoReg = 0x0066; // fractional part
constexpr uint32_t temperatureReg = 0x003D; // unlikely to read >255C
constexpr uint32_t exposureAnalogGainReg = 0x00EA;
constexpr uint32_t exposureDigitalGainCh0Reg = 0x00EC;
constexpr uint32_t
constexpr uint32_t
constexpr uint32_t frameLengthRegA = 0x050C; // msbyte
constexpr uint32_t frameLengthRegB = 0x050D;
constexpr uint32_t frameLengthRegC = 0x050E;
constexpr uint32_t frameLengthRegD = 0x050F; // lsbyte
constexpr uint32_t lineLengthHiReg = 0x0064;
constexpr uint32_t lineLengthLoReg = 0x0065;
constexpr std::initializer_list<uint32_t> registerList = 0;
*/

class CamHelperVd55G1 : public CamHelper
{
public:
	CamHelperVd55G1();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	// void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	//std::pair<uint32_t, uint32_t> getBlanking(Duration &exposure, Duration minFrameDuration,
	// 					  Duration maxFrameDuration) const override;
	//bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 22; // unknown; what is this?
};

CamHelperVd55G1::CamHelperVd55G1()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperVd55G1::gainCode(double gain) const
{
	return static_cast<uint32_t>(32 - 32 / gain);
}

double CamHelperVd55G1::gain(uint32_t gainCode) const
{
	return 32.0 / (32 - gainCode);
}

static CamHelper *create()
{
	return new CamHelperVd55G1();
}

static RegisterCamHelper reg("vd55g1_mono", &create);
