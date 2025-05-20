#pragma once

#include <print>
#include <algorithm>
#include <bit>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <exception>
#include <memory>
#include <ranges>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <system_error>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "utils.hpp"
#include "adc.hpp"
#include "asio/system_error.hpp"
#include "connect.hpp"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "gpio_cxx.hpp"
#include "hal/adc_types.h"
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

struct SamplesPerPocketRequest final {
    std::array< std::byte, 1 > type;
    std::array< std::byte, 4 > samplesNum;
};

inline constexpr auto MAX_SIZE_RX = maxSizeOfTypes< SamplingRateRequest, SamplesPerPocketRequest >();

enum class BitsPerElement : std::uint8_t { e8bit = 8, e16bit = 16, e32bit = 32 };

struct OscDataResponseHeader final {
    std::array< std::byte, 1 > cmd;
    std::array< std::byte, 1 > bitsPerElement;
    std::array< std::byte, 1 > dataPerElement;
    std::array< std::byte, 4 > dataSize;
};

[[nodiscard]] constexpr OscDataResponseHeader
oscDataResponseHeaderToRaw( BitsPerElement bpe, std::uint8_t dataPerElement, std::uint32_t dataSize ) noexcept {
    return { { std::byte( RequestType::eData ) },
             { std::byte( bpe ) },
             { std::byte( dataPerElement ) },
             toBigEndianBytes< std::byte >( dataSize ) };
}

class Oscilloscope final {
public:
    using SamplingRateType = idf::Frequency;
    using OutputDataType   = std::int16_t;
    using Adc              = core::Periph::Adc;
    using AdcHandler       = Adc::Continuous;

    enum class Atten : std::uint8_t { eX1, eX10, eX100 };

    struct CreateInfo final {
        Atten            atten;
        SamplingRateType samplingRateHz;
    };

    Oscilloscope() = default;
    explicit Oscilloscope( const CreateInfo & c, std::shared_ptr< Connect::TransferProtocol > transferProtocol ) :
    mAtten( c.atten ), mSamplingRateHZ( c.samplingRateHz ) {}

    void start();
    void stop( bool stopReciever = false ) {
        if ( mTransmitterThread.joinable() ) {
            mTransmitterThread.request_stop();
            mTransmitterThread.join();
        }

        if ( mIsRunning )
            mAdc.stop();

        mIsRunning = false;

        if ( stopReciever && mRecieverThread.joinable() ) {
            mRecieverThread.request_stop();
            mRecieverThread.join();
        }
    }

    void setTransferProtocol( std::shared_ptr< Connect::TransferProtocol > other ) {
        // stop();
        mNetProto = other;
        // start();
    }

    struct OscInfo final {
        std::shared_ptr< Connect::TransferProtocol > mNetProto;
        // BitsPerElement                               mBpe;
        // std::uint8_t                                 mDpe;
    };

    OscInfo getInfo() const { return { mNetProto, /*mBpe, mDpe*/ }; }

private:
    // NOTE: for history...
    //
    // constexpr std::uint32_t nearestSamples( std::uint32_t leastSamples ) noexcept {
    //     // auto samples = mSamplesPerPocket * Adc::Caps::digiResultBytes; -- WARNING: it is UB
    //     auto samples = leastSamples * Adc::Caps::digiResultBytes;
    //     return ( samples + ( samples % Adc::Caps::digiDataBytesPerConv ) ) / Adc::Caps::digiDataBytesPerConv;
    // }

    bool mIsRunning {};

    static constexpr std::uint32_t nearestBytes( std::uint32_t leastSamples ) noexcept {
        // NOTE: for history...
        //
        // auto bytes = leastSamples * Adc::Caps::digiResultBytes;
        // if ( bytes % Adc::Caps::digiDataBytesPerConv )
        //     bytes = ( ( bytes + Adc::Caps::digiDataBytesPerConv ) / Adc::Caps::digiDataBytesPerConv ) *
        //             Adc::Caps::digiDataBytesPerConv;
        //
        // bytes = ( bytes / Adc::Caps::digiDataBytesPerConv ) * Adc::Caps::digiDataBytesPerConv +
        //         bool( bytes % Adc::Caps::digiDataBytesPerConv ) * Adc::Caps::digiDataBytesPerConv;

        return asLeastNearestMultiple( leastSamples * Adc::Caps::digiResultBytes, Adc::Caps::digiDataBytesPerConv );
    }

    std::jthread mTransmitterThread;
    std::jthread mRecieverThread;

    using enum Atten;
    using enum BitsPerElement;

    [[maybe_unused]] Atten mAtten { eX1 };

    SamplingRateType mSamplingRateHZ { Adc::Caps::sampleFreqThresLow };

    std::uint32_t mSamplesPerPocket { nearestBytes( 512 ) / Adc::Caps::digiResultBytes };
    std::uint32_t mBytesPerPocket { nearestBytes( mSamplesPerPocket ) };

    idf::GPIONum mSignal { 36 };
    idf::GPIONum mVirtCommon { 39 };

    std::vector< adc_digi_pattern_config_t > mDigiPatterns {
        adc_digi_pattern_config_t { static_cast< std::uint8_t >( adc_atten_t::ADC_ATTEN_DB_12 ),
                                    static_cast< std::uint8_t >( AdcHandler::ioToChannel( mSignal ).channel ),
                                    static_cast< std::uint8_t >( AdcHandler::ioToChannel( mSignal ).unit ),
                                    static_cast< std::uint8_t >( adc_bitwidth_t::ADC_BITWIDTH_12 ) },
        adc_digi_pattern_config_t { static_cast< std::uint8_t >( adc_atten_t::ADC_ATTEN_DB_12 ),
                                    static_cast< std::uint8_t >( AdcHandler::ioToChannel( mVirtCommon ).channel ),
                                    static_cast< std::uint8_t >( AdcHandler::ioToChannel( mVirtCommon ).unit ),
                                    static_cast< std::uint8_t >( adc_bitwidth_t::ADC_BITWIDTH_12 ) }
    };

    AdcHandler mAdc =
    core::Periph::Adc::createContinuous( { .max_store_buf_size = mBytesPerPocket * 4 * mDigiPatterns.size(),
                                           .conv_frame_size    = mBytesPerPocket * mDigiPatterns.size(),
                                           .flags              = { .flush_pool = true } } );

    std::shared_ptr< Connect::TransferProtocol > mNetProto { nullptr };

    // BitsPerElement mBpe { e16bit };
    // std::uint8_t   mDpe { 12 };

    std::vector< adc_digi_output_data_t > mData =
    std::vector< adc_digi_output_data_t >( mSamplesPerPocket * mDigiPatterns.size() );

    std::vector< OutputDataType > mValues = std::vector< OutputDataType >( mSamplesPerPocket );
};

inline void Oscilloscope::start() {
    if ( !mNetProto )
        throw std::runtime_error( "-----Network Protocol not configured!!!\n" );

    if ( mDigiPatterns.empty() )
        throw std::runtime_error( "-----Osc is not configured!!!\n" );

    mIsRunning = true;

    mAdc.configure( mDigiPatterns, mSamplingRateHZ.get_value(), ADC_CONV_SINGLE_UNIT_1, ADC_DIGI_OUTPUT_FORMAT_TYPE1 );
    mAdc.start();

    if ( mTransmitterThread.joinable() )
        return;

    mTransmitterThread = std::jthread( [ this ]( std::stop_token token ) noexcept {
        // mData.reserve( mBytesPerPocket );

        auto transmitter = mNetProto;

        using core::Periph::AdcCali;

        switch ( AdcCali::schemeLineFittingCheckEfuse() ) {
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF:
            std::println( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF----" );
            assert( false && "TODO" );
            break;
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP:
            std::println( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP----" );
            break;
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF:
            std::println( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF----" );
            break;
        };

        auto caliHandler = AdcCali::create(
        adc_cali_line_fitting_config_t { static_cast< adc_unit_t >( mDigiPatterns.at( 0 ).unit ),
                                         static_cast< adc_atten_t >( mDigiPatterns.at( 0 ).atten ),
                                         static_cast< adc_bitwidth_t >( mDigiPatterns.at( 0 ).bit_width ),
                                         0 } );

        std::span dataSpan( mData );

        const auto sigChannel  = mDigiPatterns[ 0 ].channel;
        const auto vcomChannel = mDigiPatterns[ 1 ].channel;

        while ( !token.stop_requested() ) {
            try {
                using namespace std::chrono_literals;
                while ( !token.stop_requested() && !transmitter->hasConnection() )
                    std::this_thread::sleep_for( 1s );

                const auto resultNum =
                mAdc.read( std::as_writable_bytes( dataSpan ), std::chrono::milliseconds( ADC_MAX_DELAY ) ).resultNum;

                auto calcValueFn = [ &caliHandler ]( auto el ) -> std::int16_t {
                    return caliHandler.rawToVoltage( el.type1.data );
                };

                auto digiesSig =
                dataSpan | std::views::as_const |
                std::views::filter( [ sigChannel ]( auto & el ) { return el.type1.channel == sigChannel; } ) |
                std::views::transform( calcValueFn );

                auto digiesVcom =
                dataSpan | std::views::as_const |
                std::views::filter( [ vcomChannel ]( auto & el ) { return el.type1.channel == vcomChannel; } ) |
                std::views::transform( calcValueFn );

                std::span values( mValues.begin(), resultNum / 2 );
#define MERGE_VAR 3
#if MERGE_VAR == 1
                auto sigIt  = digiesSig.begin();
                auto vcomIt = digiesVcom.begin();

                for ( decltype( auto ) v : values ) {
                    v = static_cast< std::int16_t >( *sigIt - *vcomIt );
                    sigIt++;
                    vcomIt++;
                }

#elif MERGE_VAR == 2
                std::ranges::generate(
                values, [ sigIt = digiesSig.begin(), vcomIt = digiesVcom.begin(), &caliHandler ] mutable {
                    const auto ret = static_cast< std::int16_t >( caliHandler.rawToVoltage( sigIt->type1.data ) -
                                                                  caliHandler.rawToVoltage( vcomIt->type1.data ) );
                    sigIt++;
                    vcomIt++;
                    return ret;
                } );

#elif MERGE_VAR == 3
                std::ranges::transform( digiesSig, digiesVcom, values.begin(), std::minus(), {}, {} );
#endif
#undef MERGE_VAR

#if 0
                transmitter->write( dataSpan, []( asio::error_code ec, std::size_t transfered ) {
                    if ( ec )
                        std::print( "------ASIO Error {} !!!\n", ec.message() );

                    std::print( "-------Transfered {} bytes.\n", transfered );
                } );
#else
                transmitter->write( std::as_bytes( values ) );
#endif

            } catch ( const asio::system_error & e ) { stop(); } catch ( const std::bad_alloc & e ) {
                std::println( "------Sending error: {}", e.what() );
                transmitter->waitForDone();
            } catch ( const std::exception & e ) { std::print( "------Sending error: {}\n", e.what() ); }
        }
    } );

    if ( mRecieverThread.joinable() )
        return;

    mRecieverThread = std::jthread( [ this ]( std::stop_token token ) noexcept {
        auto reciever = mNetProto;
        while ( !token.stop_requested() ) {
            try {
                using namespace std::chrono_literals;

                std::this_thread::sleep_for( 100ms );

                const auto reciveBuffer = reciever->read( MAX_SIZE_RX );

                if ( reciveBuffer.empty() )
                    continue;

                auto firstByte = static_cast< RequestType >( reciveBuffer.front() );

                switch ( firstByte ) {
                case RequestType::eSamplesPerPocket: {
                    if ( reciveBuffer.size() < sizeof( SamplesPerPocketRequest ) )
                        throw std::runtime_error( "[Data is incorrect]" );

                    const std::span< const SamplesPerPocketRequest, 1 > reqv(
                    reinterpret_cast< const SamplesPerPocketRequest * >( reciveBuffer.data() ), 1 );

                    stop();

                    mSamplesPerPocket =
                    nearestBytes( fromBigEndianBytes( reqv.front().samplesNum ) ) / Adc::Caps::digiResultBytes;

                    mBytesPerPocket = nearestBytes( mSamplesPerPocket );

                    mData.resize( mSamplesPerPocket * mDigiPatterns.size() );

                    mValues.resize( mSamplesPerPocket );

                    mAdc = core::Periph::Adc::createContinuous(
                    { .max_store_buf_size = mBytesPerPocket * 4 * mDigiPatterns.size(),
                      .conv_frame_size    = mBytesPerPocket * mDigiPatterns.size(),
                      .flags              = { .flush_pool = true } } );

                    start();
                    std::println( "------Current samplesPerPocket: ", mSamplesPerPocket );

                    break;
                }
                case RequestType::eSamplingRate: {
                    if ( reciveBuffer.size() < sizeof( SamplingRateRequest ) )
                        throw std::runtime_error( "[Data is incorrect]" );

                    const std::span< const SamplingRateRequest, 1 > reqv(
                    reinterpret_cast< const SamplingRateRequest * >( reciveBuffer.data() ), 1 );

                    stop();
                    mSamplingRateHZ.Hz( fromBigEndianBytes( reqv.front().rateHz ) );
                    start();
                    std::println( "------Current samplingRateHz: ", mSamplingRateHZ.get_value() );

                    break;
                }
				default: throw std::runtime_error("------Wrong request type!!!");
                }

            } catch ( const asio::system_error & e ) {
                std::println( "------Reciving system error: {}", e.what() );
                stop();
            } catch ( const std::bad_alloc & e ) {
                std::println( "------Reciving bad_alloc error: {}", e.what() );
            } catch ( const std::exception & e ) { std::println( "------Reciving error: {}", e.what() ); }
        }
    } );
}

}   // namespace Osc
