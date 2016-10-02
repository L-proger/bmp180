/*
 * bmp_180.h
 *
 *  Created on: 2 окт. 2016 г.
 *      Author: l-pro
 */

#ifndef BMP180_H_
#define BMP180_H_

#include <cstdint>
#include <cmath>

namespace Bmp180Registers {
	static constexpr uint8_t Control = 0xF4;
	static constexpr uint8_t TempData = 0xF6;
	static constexpr uint8_t PressureData = 0xF6;
	static constexpr uint8_t SoftReset = 0xE0;
	static constexpr uint8_t ChipID = 0xD0;

	static constexpr uint8_t Ac1 = 0xAA;
	static constexpr uint8_t Ac2 = 0xAC;
	static constexpr uint8_t Ac3 = 0xAE;
	static constexpr uint8_t Ac4 = 0xB0;
	static constexpr uint8_t Ac5 = 0xB2;
	static constexpr uint8_t Ac6 = 0xB4;
	static constexpr uint8_t B1 = 0xB6;
	static constexpr uint8_t B2 = 0xB8;
	static constexpr uint8_t Mb = 0xBA;
	static constexpr uint8_t Mc = 0xBC;
	static constexpr uint8_t Md = 0xBE;
}

namespace Bmp180Commands {
	static constexpr uint8_t ReadTempCmd = 0x2E;
	static constexpr uint8_t ReadPressureCmd = 0x34;
}

enum class Bmp180Oversampling : uint8_t {
	UltraLowPower = 0,
	Standard = 1,
	Highres = 2,
	UltraHighRes = 3
};

struct Bmp180CalibrationData {
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
};

template<typename PlatformInterface>
class Bmp180 {
public:
	static constexpr uint8_t I2CAddress = 0x77 << 1;
	static constexpr uint8_t ChipID = 0x55;

	static constexpr float mmHgToPa(float mmHg){
		return mmHg * 133.3223684f;
	}

	static constexpr float paToMmHg(float pa){
		return pa / 133.3223684f;
	}

	static int32_t calcB5(uint16_t rawTemperature,const Bmp180CalibrationData& data){
		int32_t UT = (int32_t)rawTemperature;
		int32_t x1 = ((UT - (int32_t)data.ac6) * (int32_t)data.ac5) / (int32_t)(1 << 15);
		int32_t x2 = ((int32_t)data.mc * (int32_t)(1 << 11)) / (x1+(int32_t)data.md);
		return x1 + x2;
	}

	static int32_t rawToTrueTemperature(uint16_t rawTemperature, const Bmp180CalibrationData& data){
		return (calcB5(rawTemperature, data) + 8) / (int32_t)(1 << 4);
	}

	static int32_t rawToTruePressure(uint16_t rawTemperature, uint32_t rawPressure, Bmp180Oversampling oversampling, const Bmp180CalibrationData& data){
		auto b5 = calcB5(rawTemperature, data);

		int32_t b6 = b5 - 4000;
		int32_t x1 = ((int32_t)data.b2 * ((b6 * b6) / (1 << 12))) / (1 << 11);
		int32_t x2 = ((int32_t)data.ac2 * b6) / (1 << 11);
		int32_t x3 = x1 + x2;
		int32_t b3 = ((((int32_t)data.ac1*4 + x3) << (uint8_t)oversampling) + 2) / 4;
		x1 = ((int32_t)data.ac3 * b6) / (1 << 13);
		x2 = (data.b1 *((b6*b6)/ (1 << 12))) / (1 << 16);
		x3 = ((x1 + x2) + 2) / (1 << 2);
		uint32_t b4 = ((uint32_t)data.ac4 * (uint32_t)(x3 + 32768)) / (1 << 15);
		uint32_t b7 = (rawPressure - b3)*(uint32_t)(50000 >> (uint8_t)oversampling);

		int32_t p = 0;
		if(b7 < 0x80000000){
			p = (b7 * 2) / b4;
		}else{
			p = (b7 / b4) * 2;
		}

		x1 = (p / (1 << 8)) * (p / (1 << 8));
		x1 = (x1 * 3038) / (1 << 16);
		x2 = ((int32_t)-7357 * p) / (1 << 16);
		return p + (x1 + x2 + (int32_t)3791) / (1 << 4);
	}

	static float pressureToAltitude(int32_t pressure, float seaLevelPressure){
		return 44330.0f * (1.0f - std::pow((float)pressure / seaLevelPressure, 1.0f / 5.255f));
	}

	bool init(){
		if (PlatformInterface::read8(I2CAddress, Bmp180Registers::ChipID) != ChipID) {
			return false;
		}

		return true;
	}

	uint16_t readRawTemperature(){
		PlatformInterface::write8(I2CAddress, Bmp180Registers::Control, Bmp180Commands::ReadTempCmd);
		PlatformInterface::delayUs(4500);
		uint16_t result;
		PlatformInterface::read16(I2CAddress, Bmp180Registers::TempData, &result);
		return result;
	}

	uint32_t readRawPressure(Bmp180Oversampling oversampling){
		PlatformInterface::write8(I2CAddress, Bmp180Registers::Control, Bmp180Commands::ReadPressureCmd | ((uint8_t)oversampling << 6));

		if(oversampling == Bmp180Oversampling::UltraLowPower){
			PlatformInterface::delayUs(4500);
		}else if(oversampling == Bmp180Oversampling::Standard){
			PlatformInterface::delayUs(8000);
		}else if(oversampling == Bmp180Oversampling::Highres){
			PlatformInterface::delayUs(14000);
		}else{
			PlatformInterface::delayUs(26000);
		}

		uint32_t pressure = 0;
		PlatformInterface::read16(I2CAddress, Bmp180Registers::PressureData, reinterpret_cast<uint16_t*>(&pressure));
		pressure = (pressure << 8) | PlatformInterface::read8(I2CAddress, Bmp180Registers::PressureData + 2);
		pressure >>= (8 - (uint8_t)(oversampling));
		return pressure;
	}

	void readCalibrationData(Bmp180CalibrationData& data){
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Ac1, reinterpret_cast<uint16_t*>(&data.ac1));
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Ac2, reinterpret_cast<uint16_t*>(&data.ac2));
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Ac3, reinterpret_cast<uint16_t*>(&data.ac3));
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Ac4, &data.ac4);
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Ac5, &data.ac5);
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Ac6, &data.ac6);
		PlatformInterface::read16(I2CAddress, Bmp180Registers::B1, reinterpret_cast<uint16_t*>(&data.b1));
		PlatformInterface::read16(I2CAddress, Bmp180Registers::B2, reinterpret_cast<uint16_t*>(&data.b2));
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Mb, reinterpret_cast<uint16_t*>(&data.mb));
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Mc, reinterpret_cast<uint16_t*>(&data.mc));
		PlatformInterface::read16(I2CAddress, Bmp180Registers::Md, reinterpret_cast<uint16_t*>(&data.md));
	}
};



#endif /* BMP180_H_ */
