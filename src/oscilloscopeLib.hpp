#pragma once

#include <bit>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <new>
#include <ranges>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "adc.hpp"
#include "connect.hpp"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "gpio_cxx.hpp"
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

constexpr auto asLeastNearestMultiple( std::integral auto lowestThreshold, std::integral auto n )
-> std::common_type_t< decltype( lowestThreshold ), decltype( n ) > {
    return ( ( lowestThreshold + n - 1 ) / n ) * n;
}

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
        // auto bytes = leastSamples * Adc::Caps::digiResultBytes;
        // if ( bytes % Adc::Caps::digiDataBytesPerConv )
        //     bytes = ( ( bytes + Adc::Caps::digiDataBytesPerConv ) / Adc::Caps::digiDataBytesPerConv ) *
        //             Adc::Caps::digiDataBytesPerConv;

        // bytes = ( bytes / Adc::Caps::digiDataBytesPerConv ) * Adc::Caps::digiDataBytesPerConv +
        //         bool( bytes % Adc::Caps::digiDataBytesPerConv ) * Adc::Caps::digiDataBytesPerConv;

        return asLeastNearestMultiple( leastSamples * Adc::Caps::digiResultBytes, Adc::Caps::digiDataBytesPerConv );
    }

    std::jthread cbThread;

    using enum Atten;
    using enum BitsPerElement;

    [[maybe_unused]] Atten mAtten { eX1 };
    SamplingRateType       mSamplingRateHZ { Adc::Caps::sampleFreqThresLow };

    std::uint32_t mSamplesPerPocket { nearestBytes( 512 ) / Adc::Caps::digiResultBytes };
    std::uint32_t mBytesPerPocket { nearestBytes( mSamplesPerPocket ) };

    idf::GPIONum mSignal { 36 };
    idf::GPIONum mVirtCommon { 39 };

    std::vector< adc_digi_pattern_config_t > mDigiPatterns {
        adc_digi_pattern_config_t { static_cast< std::uint8_t >( adc_atten_t::ADC_ATTEN_DB_12 ),
                                    static_cast< std::uint8_t >( Adc::Continuous::ioToChannel( mSignal ).channel ),
                                    static_cast< std::uint8_t >( Adc::Continuous::ioToChannel( mSignal ).unit ),
                                    static_cast< std::uint8_t >( adc_bitwidth_t::ADC_BITWIDTH_12 ) },
        adc_digi_pattern_config_t { static_cast< std::uint8_t >( adc_atten_t::ADC_ATTEN_DB_12 ),
                                    static_cast< std::uint8_t >( Adc::Continuous::ioToChannel( mVirtCommon ).channel ),
                                    static_cast< std::uint8_t >( Adc::Continuous::ioToChannel( mVirtCommon ).unit ),
                                    static_cast< std::uint8_t >( adc_bitwidth_t::ADC_BITWIDTH_12 ) }
    };

    AdcHandler mAdc =
    core::Periph::Adc::createContinuous( { .max_store_buf_size = mBytesPerPocket * 4 * mDigiPatterns.size(),
                                           .conv_frame_size    = mBytesPerPocket * mDigiPatterns.size(),
                                           .flags              = { .flush_pool = true } } );

    std::shared_ptr< Connect::TransferProtocol > mNetProto { nullptr };

    BitsPerElement mBpe { e16bit };
    std::uint8_t   mDpe { 12 };

    std::vector< adc_digi_output_data_t > mData =
    std::vector< adc_digi_output_data_t >( mSamplesPerPocket * mDigiPatterns.size() );

    std::vector< std::int16_t > mValues = std::vector< std::int16_t >( mSamplesPerPocket );
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

        switch ( AdcCali::schemeLineFittingCheckEfuse() ) {
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF:
            std::print( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF----\n" );
            assert( false && "TODO" );
            break;
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP:
            std::print( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP----\n" );
            break;
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF:
            std::print( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF----\n" );
            break;
        };

#if 0
        std::vector< decltype( AdcCali::create( adc_cali_line_fitting_config_t {} ) ) > caliHandlers;
        for ( auto const pattern : mDigiPatterns )
            caliHandlers.emplace_back( AdcCali::create( { static_cast< adc_unit_t >( pattern.unit ),
                                                          static_cast< adc_atten_t >( pattern.atten ),
                                                          static_cast< adc_bitwidth_t >( pattern.bit_width ),
                                                          0 } ) );
#else
        auto caliHandler = AdcCali::create(
        adc_cali_line_fitting_config_t { static_cast< adc_unit_t >( mDigiPatterns.at( 0 ).unit ),
                                         static_cast< adc_atten_t >( mDigiPatterns.at( 0 ).atten ),
                                         static_cast< adc_bitwidth_t >( mDigiPatterns.at( 0 ).bit_width ),
                                         0 } );
#endif

        std::span dataSpan( mData );

        const auto sigChannel  = mDigiPatterns[ 0 ].channel;
        const auto vcomChannel = mDigiPatterns[ 1 ].channel;

        while ( !token.stop_requested() ) {
            try {
                using namespace std::chrono_literals;
                while ( !token.stop_requested() && !mNetProto->hasConnection() )
                    std::this_thread::sleep_for( 1s );

                const auto resultNum =
                mAdc.read( std::as_writable_bytes( dataSpan ), std::chrono::milliseconds( ADC_MAX_DELAY ) ) /
                Adc::Caps::digiResultBytes;

                auto digiesSig = dataSpan | std::views::as_const | std::views::filter( [ sigChannel ]( auto & el ) {
                                     return el.type1.channel == sigChannel;
                                 } );

                auto digiesVcom = dataSpan | std::views::as_const | std::views::filter( [ vcomChannel ]( auto & el ) {
                                      return el.type1.channel == vcomChannel;
                                  } );

                // const std::span values( mValues.begin(), resultNum );

                auto sigIt  = digiesSig.begin();
                auto vcomIt = digiesVcom.begin();

                std::span values( mValues.begin(), resultNum / 2 );
                for ( decltype( auto ) v : values ) {
#if 1
                    v = static_cast< std::int16_t >( caliHandler.rawToVoltage( sigIt->type1.data ) -
                                                     caliHandler.rawToVoltage( vcomIt->type1.data ) );
#else
                    v = static_cast< std::int16_t >( caliHandler.rawToVoltage(
                    static_cast< int >( sigIt->type1.data ) - static_cast< int >( vcomIt->type1.data ) ) );
#endif
                    sigIt++;
                    vcomIt++;
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
