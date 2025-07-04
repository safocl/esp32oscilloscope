#pragma once

#include <concepts>
#include <esp_exception.hpp>
#include <driver/dac_oneshot.h>
#include <driver/dac_cosine.h>
#include <driver/dac_continuous.h>

#include <type_traits>
#include <memory>

namespace core::Periph {

class DacOneShot final {
private:
    class Handler final {
        using DacNativeHandlerType = std::remove_pointer_t< dac_oneshot_handle_t >;
        using Deleter = decltype( []( dac_oneshot_handle_t p ) { CHECK_THROW( dac_oneshot_del_channel( p ) ); } );
        Handler( const dac_oneshot_config_t & c ) {
            CHECK_THROW( dac_oneshot_new_channel( &c, std::out_ptr( mHandle ) ) );
        };

    public:
        void outputVoltage( std::uint8_t digiValue ) {
            CHECK_THROW( dac_oneshot_output_voltage( mHandle.get(), digiValue ) );
        }
        void release() { mHandle.reset(); }

    private:
        friend DacOneShot;
        std::unique_ptr< DacNativeHandlerType, Deleter > mHandle;
    };

public:
    static Handler create( const dac_oneshot_config_t & c ) { return Handler( c ); }
};

class DacCosine final {
private:
    template < std::invocable< dac_cosine_handle_t > Deleter > class Handler final {
        using DacNativeHandlerType = std::remove_pointer_t< dac_cosine_handle_t >;
        Handler( const dac_cosine_config_t & c ) {
            CHECK_THROW( dac_cosine_new_channel( &c, std::out_ptr( mHandle ) ) );
        };

    public:
        void start() { CHECK_THROW( dac_cosine_start( mHandle.get() ) ); }
        void stop() { CHECK_THROW( dac_cosine_stop( mHandle.get() ) ); }
        void release() { mHandle.reset(); }

    private:
        friend DacCosine;
        std::unique_ptr< DacNativeHandlerType, Deleter > mHandle;
    };

public:
    using GeneralHandler =
    Handler< decltype( []( dac_cosine_handle_t p ) { CHECK_THROW( dac_cosine_del_channel( p ) ); } ) >;
    static GeneralHandler create( const dac_cosine_config_t & c ) { return GeneralHandler( c ); }

    using AutoStopHandler = Handler< decltype( []( dac_cosine_handle_t p ) {
        CHECK_THROW( dac_cosine_stop( p ) );
        CHECK_THROW( dac_cosine_del_channel( p ) );
    } ) >;
    static AutoStopHandler createWithAutoStop( const dac_cosine_config_t & c ) { return AutoStopHandler( c ); }
};
}   // namespace core::Periph
