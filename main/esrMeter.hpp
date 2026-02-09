#pragma once

#include <cmath>
#include <algorithm>
#include <bit>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <exception>
#include <memory>
#include <print>
#include <ranges>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <system_error>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "adc.hpp"
#include "connect.hpp"
#include "dac.hpp"
#include "utils.hpp"

#include <asio/system_error.hpp>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <gpio_cxx.hpp>
#include <hal/adc_types.h>
#include <hal/dac_types.h>
#include <system_cxx.hpp>

/***
 * Capacitor calculator: https://www.chipdip.ru/calc/capacitor-reactance
 * C = 1/(2*pi*f*R)
 */

/***
 * Scheme:
 * DAC_voltage 0------*----[==]----*----||----*----0 GND
 *                    |            |          |    |
 *                     \__________/ \________/     |
 *                     RrefVoltage   CxVoltage    GND
 */

namespace Esr {

enum class RequestType : std::uint8_t {
    eData             = 1,
    eStart            = 2,
    eStop             = 3,
    ePause            = 4,
    eResume           = 5,
    eSamplingRate     = 6,
    eAuto             = 7,
    eAtten            = 8,
    eSamplesPerPocket = 9,
};

struct SamplingRateRequest final {
    std::array< std::byte, 1 > type;
    std::array< std::byte, 4 > rateHz;
};
static_assert( std::is_standard_layout_v< SamplingRateRequest > );
static_assert( std::alignment_of_v< SamplingRateRequest > == 1 );
static_assert( sizeof( SamplingRateRequest ) == 5 );

struct SamplesPerPocketRequest final {
    std::array< std::byte, 1 > type;
    std::array< std::byte, 4 > samplesNum;
};
static_assert( std::is_standard_layout_v< SamplesPerPocketRequest > );
static_assert( std::alignment_of_v< SamplesPerPocketRequest > == 1 );
static_assert( sizeof( SamplesPerPocketRequest ) == 5 );

inline constexpr auto MAX_SIZE_RX = maxSizeOfTypes< SamplingRateRequest, SamplesPerPocketRequest >();

enum class BitsPerElement : std::uint8_t { e8bit = 8, e16bit = 16, e32bit = 32 };

struct EsrDataResponseHeader final {
    std::array< std::byte, 1 > cmd;
    std::array< std::byte, 1 > bitsPerElement;
    std::array< std::byte, 1 > dataPerElement;
    std::array< std::byte, 4 > dataSize;
};
static_assert( std::is_standard_layout_v< EsrDataResponseHeader > );
static_assert( std::alignment_of_v< EsrDataResponseHeader > == 1 );
static_assert( sizeof( EsrDataResponseHeader ) == 7 );

[[nodiscard]] constexpr EsrDataResponseHeader
esrDataResponseHeaderToRaw( BitsPerElement bpe, std::uint8_t dataPerElement, std::uint32_t dataSize ) noexcept {
    return { { std::byte( RequestType::eData ) },
             { std::byte( bpe ) },
             { std::byte( dataPerElement ) },
             toBigEndianBytes< std::byte >( dataSize ) };
}

inline constexpr std::uint32_t nearestBytes( std::uint32_t leastSamples ) noexcept {
    return asLeastNearestMultiple( leastSamples, core::Periph::Adc::Caps::digiDataBytesPerConv );
}

class EsrMeter final {
public:
    using FreqType       = idf::Frequency;
    using OutputDataType = std::int16_t;
    using Adc            = core::Periph::Adc;
    using AdcHandler     = Adc::Continuous;
    using AdcCali        = core::Periph::AdcCali;

    enum class VoltageAtten : std::underlying_type_t< dac_cosine_atten_t > {
        dB_0  = DAC_COSINE_ATTEN_DB_0, /*!< Original amplitude of the DAC cosine
                                       wave, equals to DAC_COSINE_ATTEN_DEFAULT */
        dB_6  = DAC_COSINE_ATTEN_DB_6, /*!< 1/2 amplitude of the DAC cosine wave */
        dB_12 = DAC_COSINE_ATTEN_DB_12, /*!< 1/4 amplitude of the DAC cosine wave */
        dB_18 = DAC_COSINE_ATTEN_DB_18, /*!< 1/8 amplitude of the DAC cosine wave */
    };

    struct CreateInfo final {
        idf::GPIONum signalRref { 39 };
        idf::GPIONum signalCx { 36 };

        FreqType adcLowSamplingRateHz { FreqType::KHz( 20 ) };
        FreqType adcHiSamplingRateHz { Adc::Caps::sampleFreqThresHigh };

        FreqType dacLowSinusFreqHz { FreqType::Hz( 200 ) };
        FreqType dacHiSinusFreqHz { FreqType::KHz( 20 ) };
    };

    EsrMeter() = default;
    EsrMeter( const CreateInfo & c, std::shared_ptr< Connect::TransferProtocol > transferProtocol ) :
    mAdcLowSamplingRate( c.adcLowSamplingRateHz ), mAdcHiSamplingRate( c.adcHiSamplingRateHz ),
    mDacLowSinusFreq( c.dacLowSinusFreqHz ), mDacHiSinusFreq( c.dacHiSinusFreqHz ), mNetProto( transferProtocol ) {}

    void start();
    void stop( bool stopReciever = false ) {
        if ( mTransmitterThread.joinable() ) {
            mTransmitterThread.request_stop();
            mTransmitterThread.join();
        }
    }

    void setTransferProtocol( std::shared_ptr< Connect::TransferProtocol > other ) {
        // stop();
        mNetProto = other;
        // start();
    }

    struct EsrMeterInfo final {
        std::shared_ptr< Connect::TransferProtocol > mNetProto;
        // BitsPerElement                               mBpe;
        // std::uint8_t                                 mDpe;
    };

    EsrMeterInfo getInfo() const { return { mNetProto, /*mBpe, mDpe*/ }; }

private:
    // static inline constexpr std::uint32_t nearestBytes( std::uint32_t leastSamples ) noexcept {
    //     return asLeastNearestMultiple( leastSamples, Adc::Caps::digiDataBytesPerConv );
    // }

    std::jthread mTransmitterThread;

    Adc::Continuous::SamplingRate mAdcLowSamplingRate { FreqType::KHz( 20 ) };
    Adc::Continuous::SamplingRate mAdcHiSamplingRate { Adc::Caps::sampleFreqThresHigh };

    FreqType mDacLowSinusFreq { FreqType::Hz( 200 ) };
    FreqType mDacHiSinusFreq { FreqType::KHz( 20 ) };

    static inline constexpr std::uint32_t mSamplesPerChannel { 512 };

    idf::GPIONum mSignalRref { 39 };
    idf::GPIONum mSignalCx { 36 };

    VoltageAtten mDacCosineAtten { VoltageAtten::dB_18 };

    // observed minimum value by ADC is 75mv. DAC offset should be incremented to +4
    // 128 / (3000mv / 75mv) = 3.2 (4 with a rounding)
    std::int8_t mDacCosineOffset { -( 128 * 7 / 8 - 8 ) };

    std::vector< AdcHandler::AdcDigiPatternConfig > mDigiPatterns {
        { adc_atten_t::ADC_ATTEN_DB_0, mSignalRref, adc_bitwidth_t::ADC_BITWIDTH_12 },
        { adc_atten_t::ADC_ATTEN_DB_0, mSignalCx, adc_bitwidth_t::ADC_BITWIDTH_12 },
    };

    std::shared_ptr< Connect::TransferProtocol > mNetProto { nullptr };

    std::vector< adc_continuous_data_t > mData =
    std::vector< adc_continuous_data_t >( mSamplesPerChannel * mDigiPatterns.size() );

    std::vector< OutputDataType > mValues =
    std::vector< OutputDataType >( mSamplesPerChannel * mDigiPatterns.size() * 3 );
};

}   // namespace Esr
