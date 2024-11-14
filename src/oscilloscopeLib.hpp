#pragma once

#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <new>
#include <ranges>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

#include "adc.hpp"
#include "connect.hpp"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include "print"
#include "system_cxx.hpp"

/*
  *
  *struct adc_continuous_evt_cbs_t
  *
  *    Group of ADC continuous mode callbacks.
  *
  *    Note
  *    These callbacks are all running in an ISR environment.
  *
  *    Note
  *    When CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE is enabled, the callback itself and functions called by it should be placed in IRAM. Involved variables should be in internal RAM as well.
  *
  *    Public Members
  *
  *    adc_continuous_callback_t on_conv_done
  *        Event callback, invoked when one conversion frame is done. See the subsection Driver Backgrounds in this header file to learn about the conversion frame concept.
  *
  *    adc_continuous_callback_t on_pool_ovf
  *        Event callback, invoked when the internal pool is full.
   */

namespace Osc {

enum class RequestType : std::uint8_t {
    eData         = 1,
    eStart        = 2,
    eStop         = 3,
    ePause        = 4,
    eResume       = 5,
    eSamplingRate = 6,
    eAuto         = 7,
    eAtten        = 8,
};

enum class BitsPerElement : std::uint8_t { e8bit = 8, e16bit = 16, e32bit = 32 };

struct OscDataResponseHeader final {
    std::array< std::uint8_t, 1 > cmd;
    std::array< std::uint8_t, 1 > bitsPerElement;
    std::array< std::uint8_t, 1 > dataPerElement;
    std::array< std::uint8_t, 4 > dataSize;
};

[[nodiscard]] constexpr inline OscDataResponseHeader
oscDataResponseHeaderToRaw( BitsPerElement bpe, std::uint8_t dataPerElement, std::uint32_t dataSize ) noexcept {
    constexpr auto shift0 = 0;
    constexpr auto shift1 = 1 * 8;
    constexpr auto shift2 = 2 * 8;
    constexpr auto shift3 = 3 * 8;

    const std::uint8_t ptr0 = static_cast< std::uint8_t >( dataSize >> shift0 );
    const std::uint8_t ptr1 = static_cast< std::uint8_t >( dataSize >> shift1 );
    const std::uint8_t ptr2 = static_cast< std::uint8_t >( dataSize >> shift2 );
    const std::uint8_t ptr3 = static_cast< std::uint8_t >( dataSize >> shift3 );

    if constexpr ( std::endian::native == std::endian::little )
        return { { std::to_underlying( RequestType::eData ) },
                 { std::to_underlying( bpe ) },
                 { dataPerElement },
                 { ptr0, ptr1, ptr2, ptr3 } };
    else if constexpr ( std::endian::native == std::endian::big )
        return { { std::to_underlying( RequestType::eData ) },
                 { std::to_underlying( bpe ) },
                 { dataPerElement },
                 { ptr3, ptr2, ptr1, ptr0 } };
    else
        return { { std::to_underlying( RequestType::eData ) },
                 { std::to_underlying( bpe ) },
                 { dataPerElement },
                 { ptr0, ptr1, ptr2, ptr3 } };
}

class Oscilloscope final {
public:
    using SamplingRateType = idf::Frequency;
    using DataType         = std::uint16_t;
    using Adc              = core::Periph::Adc;
    using AdcHandler       = Adc::Continuous;

    enum class Atten : std::uint8_t { eX1, eX10, eX100 };

    struct CreateInfo final {
        Atten       atten;
        std::size_t samplingRateHz;
    };

    Oscilloscope() = default;
    explicit Oscilloscope( CreateInfo && c, std::shared_ptr< Connect::TransferProtocol > transferProtocol ) :
    mAtten( c.atten ), mSamplingRateHZ( c.samplingRateHz ) {}

    void start();
    void stop() {
        cbThread.request_stop();
        cbThread.join();
        mAdc.stop();
    }

    void setTransferProtocol( std::shared_ptr< Connect::TransferProtocol > other ) {
        // stop();
        mNetProto = other;
        // start();
    }

    struct OscInfo final {
        std::shared_ptr< Connect::TransferProtocol > mNetProto;
        BitsPerElement                               mBpe;
        std::uint8_t                                 mDpe;
    };

    OscInfo getInfo() const { return { mNetProto, mBpe, mDpe }; }

private:
    // constexpr std::uint32_t nearestSamples( std::uint32_t leastSamples ) noexcept {
    //     // auto samples = mSamplesPerPocket * Adc::Caps::digiResultBytes; -- it is UB
    //     auto samples = leastSamples * Adc::Caps::digiResultBytes;
    //     return ( samples + ( samples % Adc::Caps::digiDataBytesPerConv ) ) / Adc::Caps::digiDataBytesPerConv;
    // }

    constexpr std::uint32_t nearestBytes( std::uint32_t leastSamples ) noexcept {
        auto bytes = leastSamples * Adc::Caps::digiResultBytes;
        if ( bytes % Adc::Caps::digiDataBytesPerConv )
            bytes = ( ( bytes + Adc::Caps::digiDataBytesPerConv ) / Adc::Caps::digiDataBytesPerConv ) *
                    Adc::Caps::digiDataBytesPerConv;
        return bytes;
    }

    std::jthread cbThread;

    using enum Atten;
    using enum BitsPerElement;

    [[maybe_unused]] Atten mAtten { eX1 };
    SamplingRateType       mSamplingRateHZ { Adc::Caps::sampleFreqThresLow };

    std::uint32_t mSamplesPerPocket { nearestBytes( 512 ) / Adc::Caps::digiResultBytes };
    std::uint32_t mBytesPerPocket { nearestBytes( mSamplesPerPocket ) };

    AdcHandler mAdc = core::Periph::Adc::createContinuous( { .max_store_buf_size = mBytesPerPocket * 4,
                                                             .conv_frame_size    = mBytesPerPocket,
                                                             .flags              = { .flush_pool = true } } );

    std::vector< adc_digi_pattern_config_t > mDigiPatterns { adc_digi_pattern_config_t {
    static_cast< std::uint8_t >( adc_atten_t::ADC_ATTEN_DB_12 ),
    static_cast< std::uint8_t >( adc_channel_t::ADC_CHANNEL_0 ),
    static_cast< std::uint8_t >( adc_unit_t::ADC_UNIT_1 ),
    static_cast< std::uint8_t >( adc_bitwidth_t::ADC_BITWIDTH_12 ) } };

    std::shared_ptr< Connect::TransferProtocol > mNetProto { nullptr };

    BitsPerElement mBpe { e16bit };
    std::uint8_t   mDpe { 12 };

    std::vector< std::byte >     mData   = std::vector< std::byte >( mBytesPerPocket );
    std::vector< std::uint16_t > mValues = std::vector< std::uint16_t >( mSamplesPerPocket );

    // void onRead() {
    //     mAdc.read( data, std::chrono::milliseconds( ADC_MAX_DELAY ) );
    //
    //     mNetProto->write( data, [ this ]( asio::error_code ec, std::size_t transfered ) {
    //         if ( ec )
    //             std::print( "------ASIO Error {} !!!\n", ec.message() );
    //         else {
    //             std::print( "-------Transfered {} bytes.\n", transfered );
    //             onRead();
    //         }
    //     } );
    // }
};

inline void Oscilloscope::start() {
    if ( !mNetProto )
        throw std::runtime_error( "-----Network Protocol not configured!!!\n" );

    if ( mDigiPatterns.empty() )
        throw std::runtime_error( "-----Osc is not configured!!!\n" );

    mAdc.configure( mDigiPatterns, mSamplingRateHZ.get_value(), ADC_CONV_SINGLE_UNIT_1, ADC_DIGI_OUTPUT_FORMAT_TYPE1 );
    mAdc.start();

    cbThread = std::jthread( [ this ]( std::stop_token token ) {
        // mData.reserve( mBytesPerPocket );

        using core::Periph::AdcCali;

        auto const                     digPattern = mDigiPatterns.front();
        adc_cali_line_fitting_config_t caliConf { .unit_id      = static_cast< adc_unit_t >( digPattern.unit ),
                                                  .atten        = static_cast< adc_atten_t >( digPattern.atten ),
                                                  .bitwidth     = static_cast< adc_bitwidth_t >( digPattern.bit_width ),
                                                  .default_vref = 0 };

        switch ( AdcCali::schemeLineFittingCheckEfuse() ) {
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF:
            caliConf.default_vref = 3300;
            std::print( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF----\n" );
            break;
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP:
            std::print( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP----\n" );
            break;
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF:
            std::print( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF----\n" );
            break;
        };

        auto caliHandler = AdcCali::create( std::move( caliConf ) );

        std::span< std::byte > dataSpan { mData };

        while ( !token.stop_requested() ) {
            try {
                using namespace std::chrono_literals;
                while ( !token.stop_requested() && !mNetProto->hasConnection() )
                    std::this_thread::sleep_for( 1s );

                const auto resultNum =
                mAdc.read( dataSpan, std::chrono::milliseconds( ADC_MAX_DELAY ) ) / Adc::Caps::digiResultBytes;

                // std::span< adc_digi_output_data_t > digiSpan ( dataSpan );

                const std::span< const adc_digi_output_data_t > digies(
                reinterpret_cast< const adc_digi_output_data_t * >( dataSpan.data() ), resultNum );

                const std::span< std::uint16_t > values( mValues.begin(), resultNum );

                auto itVal = values.begin();
                for ( auto & v : digies ) {
                    *itVal = caliHandler.rawToVoltage( v.type1.data );
                    ++itVal;
                }
#if 0
                mNetProto->write( dataSpan, []( asio::error_code ec, std::size_t transfered ) {
                    if ( ec )
                        std::print( "------ASIO Error {} !!!\n", ec.message() );

                    std::print( "-------Transfered {} bytes.\n", transfered );
                } );
#else
                mNetProto->write( std::as_bytes( values ) );
#endif

            } catch ( const std::bad_alloc & e ) {
                std::print( "------Sending error: {}\n", e.what() );
                mNetProto->waitForDone();
            } catch ( const std::exception & e ) { std::print( "------Sending error: {}\n", e.what() ); }
        }
    } );
}

}   // namespace Osc
