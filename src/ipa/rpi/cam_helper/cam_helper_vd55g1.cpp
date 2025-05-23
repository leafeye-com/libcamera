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
#include "md_parser.h"

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
constexpr uint32_t vtAnalogGainReg = 0x0057; // five bits used
constexpr uint32_t ispDigitalGainCh0HiReg = 0x0058; // integer part
constexpr uint32_t ispDigitalGainCh0LoReg = 0x0059; // fractional part
/*
 * Exposure mode control:
 * 0x0 = automatic
 * 0x1 = freeze AE with current settings
 * 0x2 = manual setting mode
 * 0x4 = bypass
 */
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
constexpr uint32_t
constexpr std::initializer_list<uint32_t> registerList = 0;




/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the Sony IMX477 datasheet:
 */
constexpr uint32_t expHiReg = 0x0202;
constexpr uint32_t expLoReg = 0x0203;
constexpr uint32_t gainHiReg = 0x0204;
constexpr uint32_t gainLoReg = 0x0205;
constexpr uint32_t frameLengthHiReg = 0x0340;
constexpr uint32_t frameLengthLoReg = 0x0341;
constexpr uint32_t lineLengthHiReg = 0x0342;
constexpr uint32_t lineLengthLoReg = 0x0343;
constexpr uint32_t temperatureReg = 0x013a;
constexpr std::initializer_list<uint32_t> registerList =
	{ expHiReg, expLoReg, gainHiReg, gainLoReg, frameLengthHiReg, frameLengthLoReg,
	  lineLengthHiReg, lineLengthLoReg, temperatureReg };

class CamHelperImx477 : public CamHelper
{
public:
	CamHelperImx477();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	std::pair<uint32_t, uint32_t> getBlanking(Duration &exposure, Duration minFrameDuration,
						  Duration maxFrameDuration) const override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 22;
	/* Maximum frame length allowable for long exposure calculations. */
	static constexpr int frameLengthMax = 0xffdc;
	/* Largest long exposure scale factor given as a left shift on the frame length. */
	static constexpr int longExposureShiftMax = 7;

	void populateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;
};

CamHelperImx477::CamHelperImx477()
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
{
}

uint32_t CamHelperImx477::gainCode(double gain) const
{
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double CamHelperImx477::gain(uint32_t gainCode) const
{
	return 1024.0 / (1024 - gainCode);
}

void CamHelperImx477::prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	MdParser::RegisterMap registers;
	DeviceStatus deviceStatus;

	if (metadata.get("device.status", deviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found from DelayedControls";
		return;
	}

	parseEmbeddedData(buffer, metadata);

	/*
	 * The DeviceStatus struct is first populated with values obtained from
	 * DelayedControls. If this reports frame length is > frameLengthMax,
	 * it means we are using a long exposure mode. Since the long exposure
	 * scale factor is not returned back through embedded data, we must rely
	 * on the existing exposure lines and frame length values returned by
	 * DelayedControls.
	 *
	 * Otherwise, all values are updated with what is reported in the
	 * embedded data.
	 */
	if (deviceStatus.frameLength > frameLengthMax) {
		DeviceStatus parsedDeviceStatus;

		metadata.get("device.status", parsedDeviceStatus);
		parsedDeviceStatus.exposureTime = deviceStatus.exposureTime;
		parsedDeviceStatus.frameLength = deviceStatus.frameLength;
		metadata.set("device.status", parsedDeviceStatus);

		LOG(IPARPI, Debug) << "Metadata updated for long exposure: "
				   << parsedDeviceStatus;
	}
}

std::pair<uint32_t, uint32_t> CamHelperImx477::getBlanking(Duration &exposure,
							   Duration minFrameDuration,
							   Duration maxFrameDuration) const
{
	uint32_t frameLength, exposureLines;
	unsigned int shift = 0;

	auto [vblank, hblank] = CamHelper::getBlanking(exposure, minFrameDuration,
						       maxFrameDuration);

	frameLength = mode_.height + vblank;
	Duration lineLength = hblankToLineLength(hblank);

	/*
	 * Check if the frame length calculated needs to be setup for long
	 * exposure mode. This will require us to use a long exposure scale
	 * factor provided by a shift operation in the sensor.
	 */
	while (frameLength > frameLengthMax) {
		if (++shift > longExposureShiftMax) {
			shift = longExposureShiftMax;
			frameLength = frameLengthMax;
			break;
		}
		frameLength >>= 1;
	}

	if (shift) {
		/* Account for any rounding in the scaled frame length value. */
		frameLength <<= shift;
		exposureLines = CamHelperImx477::exposureLines(exposure, lineLength);
		exposureLines = std::min(exposureLines, frameLength - frameIntegrationDiff);
		exposure = CamHelperImx477::exposure(exposureLines, lineLength);
	}

	return { frameLength - mode_.height, hblank };
}

bool CamHelperImx477::sensorEmbeddedDataPresent() const
{
	return true;
}

void CamHelperImx477::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;

	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(lineLengthHiReg) * 256 +
							  registers.at(lineLengthLoReg));
	deviceStatus.exposureTime = exposure(registers.at(expHiReg) * 256 + registers.at(expLoReg),
					     deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(gainHiReg) * 256 + registers.at(gainLoReg));
	deviceStatus.frameLength = registers.at(frameLengthHiReg) * 256 + registers.at(frameLengthLoReg);
	deviceStatus.sensorTemperature = std::clamp<int8_t>(registers.at(temperatureReg), -20, 80);

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	return new CamHelperImx477();
}

static RegisterCamHelper reg("imx477", &create);
