#include "adc.hpp"

namespace core::Periph {
Adc::OneShot::OneShot( InitConfig && c ) :
mHandle( []( InitConfig && c ) {
    Handle h {};
    CHECK_THROW( adc_oneshot_new_unit( &c, &h ) );
    return h;
}( std::move( c ) ) ) {}

void Adc::OneShot::congigChannel( AdcChannelNum chan, adc_oneshot_chan_cfg_t config ) {
    CHECK_THROW( adc_oneshot_config_channel( mHandle, chan.get_value(), &config ) );
}

int Adc::OneShot::getOneShotValue( AdcChannelNum chan ) {
    int v;
    CHECK_THROW( adc_oneshot_read( mHandle, chan.get_value(), &v ) );
    return v;
}

std::tuple< Adc::AdcUnitNum, Adc::AdcChannelNum > Adc::OneShot::ioToChannel( idf::GPIONum pin ) {
    adc_unit_t    unit;
    adc_channel_t channel;
    CHECK_THROW( adc_oneshot_io_to_channel( pin.get_value(), &unit, &channel ) );

    return { AdcUnitNum( unit ), AdcChannelNum( channel ) };
}

idf::GPIONum Adc::OneShot::channelToIo( std::tuple< AdcUnitNum, AdcChannelNum > info ) {
    int ret;
    CHECK_THROW( adc_oneshot_channel_to_io(
    std::get< AdcUnitNum >( info ).get_value(), std::get< AdcChannelNum >( info ).get_value(), &ret ) );

    return idf::GPIONum( ret );
}

Adc::Continuous Adc::createContinuous( Continuous::InitConfig && cfg ) { return Continuous( std::move( cfg ) ); }
Adc::Continuous
Adc::createContinuous( Continuous::Samples maxStoreBuf, Continuous::Samples convFrames, bool isFlushPool ) {
    return Continuous( { .max_store_buf_size = maxStoreBuf.get_value(),
                         .conv_frame_size    = convFrames.get_value(),
                         .flags              = { .flush_pool = isFlushPool } } );
}
Adc::OneShot Adc::createOneShot( OneShot::InitConfig && cfg ) { return OneShot( std::move( cfg ) ); }
}   // namespace core::Periph
